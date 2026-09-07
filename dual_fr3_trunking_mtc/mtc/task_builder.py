from __future__ import annotations

import math
from typing import Sequence

from geometry_msgs.msg import Pose, PoseStamped, Vector3, Vector3Stamped
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from ..gripper import GripperProfileRegistry, GripperRequest
from ..models import (
    DEFAULT_FOLLOWER_ORIENTATION_DIRECTION,
    DEFAULT_LEADER_LEAD_DISTANCE,
    DEFAULT_LEADER_ORIENTATION_DIRECTION,
    DEFAULT_TOOL_PITCH,
    DEFAULT_TOOL_ROLL,
    Keypoint,
    TaskPlan,
    rpy_to_quaternion,
)
from ..runtime.config import DEFAULTS
from ..stages.compiler import build_mtc_stage_specs, finger_joint_for_ik_frame
from ..stages.specs import MtcStageSpec


OMPL_PIPELINE_NAME = "move_group"
OMPL_PLANNER_ID = "RRTConnectkConfigDefault"
OMPL_NUM_PLANNING_ATTEMPTS = 5
OMPL_MOVE_TO_TIMEOUT = 5.0
DEFAULT_ANCHOR_MAX_PATH_Z = DEFAULTS.anchor_max_path_z
# PositionConstraint only supports bounded regions. These bounds are deliberately
# wider than the FR3 reachable workspace so only the upper TCP z limit is active.
ANCHOR_PATH_CONSTRAINT_MIN_Z = -2.0
ANCHOR_PATH_CONSTRAINT_XY_SIZE = 4.0


def import_mtc_modules():
    try:
        import rclcpp
        from moveit.task_constructor import core, stages
    except ImportError as exc:
        raise RuntimeError(
            "Python MoveIt Task Constructor is not importable. Source the MoveIt "
            "workspace before this workspace, for example: "
            "source /home/jerry/ws_moveit/install/setup.bash"
        ) from exc
    return rclcpp, core, stages


def create_motion_planners(
    core,
    node,
    cartesian_step_size: float,
    motion_velocity_scaling: float,
    motion_acceleration_scaling: float,
):
    cartesian = core.CartesianPath()
    cartesian.step_size = cartesian_step_size
    cartesian.jump_threshold = 0.0
    cartesian.max_velocity_scaling_factor = motion_velocity_scaling
    cartesian.max_acceleration_scaling_factor = motion_acceleration_scaling

    jointspace = core.JointInterpolationPlanner()
    jointspace.max_velocity_scaling_factor = motion_velocity_scaling
    jointspace.max_acceleration_scaling_factor = motion_acceleration_scaling

    ompl = core.PipelinePlanner(node, OMPL_PIPELINE_NAME)
    ompl.planner = OMPL_PLANNER_ID
    ompl.num_planning_attempts = OMPL_NUM_PLANNING_ATTEMPTS
    ompl.max_velocity_scaling_factor = motion_velocity_scaling
    ompl.max_acceleration_scaling_factor = motion_acceleration_scaling
    return cartesian, jointspace, ompl


def planner_for_move_to(spec: MtcStageSpec, cartesian, jointspace, ompl):
    planners = {
        "CartesianPath": cartesian,
        "JointInterpolationPlanner": jointspace,
        "PipelinePlanner": ompl,
    }
    try:
        return planners[spec.planner]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported MoveTo planner {spec.planner!r} for stage {spec.name!r}"
        ) from exc


def max_link_z_path_constraint(
    frame_id: str,
    link_name: str,
    max_z: float,
) -> Constraints:
    if not math.isfinite(max_z):
        raise ValueError("anchor_max_path_z must be finite")
    if max_z <= ANCHOR_PATH_CONSTRAINT_MIN_Z:
        raise ValueError(
            "anchor_max_path_z must be greater than "
            f"{ANCHOR_PATH_CONSTRAINT_MIN_Z}"
        )

    height = max_z - ANCHOR_PATH_CONSTRAINT_MIN_Z
    region = SolidPrimitive()
    region.type = SolidPrimitive.BOX
    region.dimensions = [
        ANCHOR_PATH_CONSTRAINT_XY_SIZE,
        ANCHOR_PATH_CONSTRAINT_XY_SIZE,
        height,
    ]

    region_pose = Pose()
    region_pose.position.z = ANCHOR_PATH_CONSTRAINT_MIN_Z + 0.5 * height
    region_pose.orientation.w = 1.0

    position = PositionConstraint()
    position.header.frame_id = frame_id
    position.link_name = link_name
    position.constraint_region.primitives.append(region)
    position.constraint_region.primitive_poses.append(region_pose)
    position.weight = 1.0

    constraints = Constraints()
    constraints.name = f"{link_name}_max_z_{max_z:.3f}"
    constraints.position_constraints.append(position)
    return constraints


def _identity_ik_frame(frame_id: str) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.orientation.w = 1.0
    return pose


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _pose_at_keypoint(
    keypoint: Keypoint,
    tool_roll: float,
    tool_pitch: float,
    target_yaw: float,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = keypoint.frame_id
    pose.pose.position.x = keypoint.position[0]
    pose.pose.position.y = keypoint.position[1]
    pose.pose.position.z = keypoint.position[2]
    pose.pose.orientation = rpy_to_quaternion(
        tool_roll,
        tool_pitch,
        _normalize_angle(target_yaw),
    )
    return pose


def create_mtc_task(
    node,
    task_plan: TaskPlan,
    stage_specs: Sequence[MtcStageSpec] | None = None,
    leader_group: str = "left_fr3_arm",
    follower_group: str = "right_fr3_arm",
    leader_ik_frame: str = "left_fr3_hand_tcp",
    follower_ik_frame: str = "right_fr3_hand_tcp",
    cartesian_step_size: float = 0.01,
    motion_velocity_scaling: float = 0.2,
    motion_acceleration_scaling: float = 0.2,
    leader_lead_distance: float = DEFAULT_LEADER_LEAD_DISTANCE,
    tool_roll: float = DEFAULT_TOOL_ROLL,
    tool_pitch: float = DEFAULT_TOOL_PITCH,
    selected_stage_indices: set[int] | None = None,
    leader_orientation_direction: str = DEFAULT_LEADER_ORIENTATION_DIRECTION,
    follower_orientation_direction: str = DEFAULT_FOLLOWER_ORIENTATION_DIRECTION,
    anchor_max_path_z: float = DEFAULT_ANCHOR_MAX_PATH_Z,
    gripper_profiles: GripperProfileRegistry | None = None,
):
    _rclcpp, core, stages = import_mtc_modules()
    keypoints = task_plan.keypoints

    cartesian, jointspace, ompl = create_motion_planners(
        core,
        node,
        cartesian_step_size,
        motion_velocity_scaling,
        motion_acceleration_scaling,
    )

    task = core.Task()
    task.name = "dual_fr3_trunking_mtc_prototype"
    task.loadRobotModel(node)
    task.add(stages.CurrentState("current_state"))

    specs = list(stage_specs) if stage_specs is not None else build_mtc_stage_specs(
        task_plan,
        leader_group=leader_group,
        follower_group=follower_group,
        leader_ik_frame=leader_ik_frame,
        follower_ik_frame=follower_ik_frame,
        leader_lead_distance=leader_lead_distance,
        leader_orientation_direction=leader_orientation_direction,
        follower_orientation_direction=follower_orientation_direction,
    )
    for spec in specs:
        if (
            selected_stage_indices is not None
            and spec.stage_index not in selected_stage_indices
        ):
            continue
        if not spec.executable:
            continue

        if spec.mtc_stage_type == "Merger":
            merger = core.Merger(spec.name)
            for child in spec.children:
                move = stages.MoveRelative(child.name, cartesian)
                move.group = child.group
                move.ik_frame = _identity_ik_frame(child.ik_frame)
                move.setDirection(
                    Vector3Stamped(
                        header=Header(frame_id=child.frame_id),
                        vector=Vector3(
                            x=child.vector[0],
                            y=child.vector[1],
                            z=child.vector[2],
                        ),
                    )
                )
                merger.insert(move)
            task.add(merger)
            continue

        if spec.mtc_stage_type == "GripperOperation":
            if gripper_profiles is None:
                raise ValueError(
                    f"gripper profiles are required by stage {spec.name!r}"
                )
            profile = gripper_profiles.resolve(
                GripperRequest(
                    actor=spec.actor,
                    profile=spec.gripper_profile,
                    action_override=spec.gripper_action or None,
                    width_override=spec.gripper_width_override,
                )
            )
            # Keep a hand-joint representation in the MTC solution for collision
            # checking and RViz. Staged execution sends the backend action instead.
            if profile.action != "hold":
                move_to = stages.MoveTo(spec.name, jointspace)
                move_to.group = spec.group
                move_to.setGoal(
                    {
                        finger_joint_for_ik_frame(spec.ik_frame): (
                            profile.finger_joint_position
                        )
                    }
                )
                task.add(move_to)
            continue

        if spec.mtc_stage_type == "MoveTo":
            planner = planner_for_move_to(spec, cartesian, jointspace, ompl)
            move_to = stages.MoveTo(spec.name, planner)
            if spec.planner == "PipelinePlanner":
                move_to.timeout = OMPL_MOVE_TO_TIMEOUT
            if spec.primitive == "direct_move_to_next_anchor":
                move_to.path_constraints = max_link_z_path_constraint(
                    spec.frame_id,
                    spec.ik_frame,
                    anchor_max_path_z,
                )
            move_to.group = spec.group
            if spec.primitive == "move_above_initial_keypoint":
                move_to.ik_frame = _identity_ik_frame(spec.ik_frame)
                target = _pose_at_keypoint(
                    keypoints[spec.from_index],
                    tool_roll,
                    tool_pitch,
                    spec.target_yaw,
                )
                target.pose.position.x += spec.vector[0]
                target.pose.position.y += spec.vector[1]
                target.pose.position.z += spec.vector[2]
                move_to.setGoal(target)
            elif spec.primitive == "turn_gripper_to_next_keypoint":
                move_to.ik_frame = _identity_ik_frame(spec.ik_frame)
                move_to.setGoal(
                    _pose_at_keypoint(
                        keypoints[spec.from_index],
                        tool_roll,
                        tool_pitch,
                        spec.target_yaw,
                    )
                )
            elif spec.primitive in {
                "direct_move_to_next_anchor",
                "direct_move_to_seat_edge_keypoint",
            }:
                move_to.ik_frame = _identity_ik_frame(spec.ik_frame)
                move_to.setGoal(
                    _pose_at_keypoint(
                        keypoints[spec.to_index],
                        tool_roll,
                        tool_pitch,
                        spec.target_yaw,
                    )
                )
            else:
                move_to.setGoal(dict(spec.joint_goal))
            task.add(move_to)
            continue

        move = stages.MoveRelative(spec.name, cartesian)
        move.group = spec.group
        move.ik_frame = _identity_ik_frame(spec.ik_frame)
        move.setDirection(
            Vector3Stamped(
                header=Header(frame_id=spec.frame_id),
                vector=Vector3(
                    x=spec.vector[0],
                    y=spec.vector[1],
                    z=spec.vector[2],
                ),
            )
        )
        task.add(move)

    return task, specs
