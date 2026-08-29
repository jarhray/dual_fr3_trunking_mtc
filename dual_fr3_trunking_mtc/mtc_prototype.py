from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Vector3, Vector3Stamped
from std_msgs.msg import Header
from std_msgs.msg import String

from .models import (
    DEFAULT_LEADER_LEAD_DISTANCE,
    DEFAULT_TOOL_PITCH,
    DEFAULT_TOOL_ROLL,
    Keypoint,
    TaskStep,
    rpy_to_quaternion,
)
from .planner import build_segment_plans, load_keypoints
from .scheduler import build_task_schedule


@dataclass(frozen=True)
class MtcStageSpec:
    step_index: int
    stage_index: int
    stage_key: str
    action: str
    actor: str
    group: str
    ik_frame: str
    name: str
    from_index: int
    to_index: int
    from_keypoint: str
    to_keypoint: str
    frame_id: str
    vector: tuple[float, float, float]
    execution_order: List[str]
    primitive: str
    mtc_stage_type: str
    planner: str
    executable: bool = True
    yaw_delta: float = 0.0
    target_yaw: float = 0.0
    joint_goal: Dict[str, float] = field(default_factory=dict)
    info: str = ""


def _make_logger() -> logging.Logger:
    logging.basicConfig(
        level=logging.INFO,
        format="[mtc_prototype] %(message)s",
        stream=sys.stdout,
    )
    return logging.getLogger("dual_fr3_trunking_mtc.mtc_prototype")


def _index_pair_for_step(step: TaskStep) -> tuple[str, int | None, int | None]:
    if step.leader_from_index is not None and step.leader_to_index is not None:
        return "leader", step.leader_from_index, step.leader_to_index
    if step.follower_from_index is not None and step.follower_to_index is not None:
        return "follower", step.follower_from_index, step.follower_to_index
    return "none", None, None


def _delta(start: Keypoint, goal: Keypoint) -> tuple[float, float, float]:
    if start.frame_id != goal.frame_id:
        raise ValueError(
            f"Cannot create MTC relative move between different frames: "
            f"{start.name} is in {start.frame_id}, {goal.name} is in {goal.frame_id}"
        )
    return (
        goal.position[0] - start.position[0],
        goal.position[1] - start.position[1],
        goal.position[2] - start.position[2],
    )


def _norm(vector: tuple[float, float, float]) -> float:
    return math.sqrt(sum(component * component for component in vector))


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_toward(vector: tuple[float, float, float]) -> float:
    return math.atan2(vector[1], vector[0])


def _path_direction(
    keypoints: Sequence[Keypoint],
    index: int,
) -> tuple[float, float, float]:
    if len(keypoints) < 2:
        raise ValueError("At least two keypoints are required to derive path direction")
    if not 0 <= index < len(keypoints):
        raise ValueError(f"keypoint index out of range: {index}")
    if index < len(keypoints) - 1:
        return _delta(keypoints[index], keypoints[index + 1])
    return _delta(keypoints[index - 1], keypoints[index])


def _path_yaw(keypoints: Sequence[Keypoint], index: int) -> float:
    direction = _path_direction(keypoints, index)
    if math.hypot(direction[0], direction[1]) < 1e-9:
        return 0.0
    return _yaw_toward(direction)


def _scaled_direction(
    vector: tuple[float, float, float],
    distance: float,
) -> tuple[float, float, float]:
    length = _norm(vector)
    if length < 1e-9:
        raise ValueError("Cannot scale a zero-length path direction")
    scale = distance / length
    return tuple(component * scale for component in vector)


def _identity_ik_frame(frame_id: str) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.orientation.w = 1.0
    return pose


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


def _hand_group_for_arm_group(arm_group: str) -> str:
    if arm_group.endswith("_arm"):
        return f"{arm_group[:-4]}_hand"
    return f"{arm_group}_hand"


def _finger_joint_for_ik_frame(ik_frame: str) -> str:
    if ik_frame.endswith("hand_tcp"):
        return f"{ik_frame[:-8]}finger_joint1"
    return f"{ik_frame}_finger_joint1"


def _base_spec_kwargs(
    step: TaskStep,
    actor: str,
    group: str,
    ik_frame: str,
    start: Keypoint,
    goal: Keypoint,
    from_index: int,
    to_index: int,
    vector: tuple[float, float, float],
) -> Dict[str, Any]:
    return {
        "step_index": step.index,
        "action": step.action,
        "actor": actor,
        "group": group,
        "ik_frame": ik_frame,
        "from_index": from_index,
        "to_index": to_index,
        "from_keypoint": start.name,
        "to_keypoint": goal.name,
        "frame_id": start.frame_id,
        "vector": vector,
        "execution_order": list(step.execution_order),
    }


def _stage_key(
    step: TaskStep,
    actor: str,
    start: Keypoint,
    goal: Keypoint,
    primitive: str,
) -> str:
    return (
        f"step_{step.index}:{step.action}:{actor}:"
        f"{start.name}->{goal.name}:{primitive}"
    )


def _initial_stage_key(actor: str, goal: Keypoint, primitive: str) -> str:
    return f"initial:{actor}:current->{goal.name}:{primitive}"


def _initial_gripper_close_spec(
    stage_index: int,
    actor: str,
    arm_group: str,
    ik_frame: str,
    keypoints: Sequence[Keypoint],
    goal_index: int,
) -> MtcStageSpec:
    goal = keypoints[goal_index]
    primitive = "close_gripper_at_start"
    return MtcStageSpec(
        step_index=-1,
        stage_index=stage_index,
        stage_key=_initial_stage_key(actor, goal, primitive),
        action="initialize",
        actor=actor,
        group=_hand_group_for_arm_group(arm_group),
        ik_frame=ik_frame,
        name=f"initial_{actor}_close_gripper",
        from_index=goal_index,
        to_index=goal_index,
        from_keypoint="current",
        to_keypoint=goal.name,
        frame_id=goal.frame_id,
        vector=(0.0, 0.0, 0.0),
        execution_order=[f"{actor}_close_gripper"],
        primitive=primitive,
        mtc_stage_type="MoveTo",
        planner="JointInterpolationPlanner",
        joint_goal={_finger_joint_for_ik_frame(ik_frame): 0.0},
        info="close gripper before any arm motion",
    )


def _initial_alignment_spec(
    stage_index: int,
    actor: str,
    group: str,
    ik_frame: str,
    keypoints: Sequence[Keypoint],
    goal_index: int,
    target_yaw: float,
) -> MtcStageSpec:
    goal = keypoints[goal_index]
    return MtcStageSpec(
        step_index=-1,
        stage_index=stage_index,
        stage_key=_initial_stage_key(actor, goal, "move_to_initial_keypoint"),
        action="initialize",
        actor=actor,
        group=group,
        ik_frame=ik_frame,
        name=f"initial_{actor}_move_to_{goal.name}",
        from_index=goal_index,
        to_index=goal_index,
        from_keypoint="current",
        to_keypoint=goal.name,
        frame_id=goal.frame_id,
        vector=(0.0, 0.0, 0.0),
        execution_order=[f"{actor}_move_to_initial_keypoint"],
        primitive="move_to_initial_keypoint",
        mtc_stage_type="MoveTo",
        planner="JointInterpolationPlanner",
        target_yaw=target_yaw,
        info=(
            "direct joint-space plan from current robot state to the initial "
            "trunking keypoint using the task tool orientation and path yaw"
        ),
    )


def _leader_seat_edge_turn_spec(
    stage_index: int,
    step: TaskStep,
    keypoints: Sequence[Keypoint],
    leader_group: str,
    leader_ik_frame: str,
    current_yaw: float,
) -> MtcStageSpec:
    if step.leader_hold_index is None:
        raise ValueError("seat_edge step requires leader_hold_index")
    hold_index = step.leader_hold_index
    hold = keypoints[hold_index]
    target_yaw = _path_yaw(keypoints, hold_index)
    yaw_delta = _normalize_angle(target_yaw - current_yaw)
    executable = not math.isclose(yaw_delta, 0.0, abs_tol=1e-6)
    return MtcStageSpec(
        step_index=step.index,
        stage_index=stage_index,
        stage_key=(
            f"step_{step.index}:seat_edge:leader:{hold.name}->{hold.name}:"
            "turn_gripper_to_next_keypoint"
        ),
        action="seat_edge",
        actor="leader",
        group=leader_group,
        ik_frame=leader_ik_frame,
        name=f"step_{step.index}_seat_edge_leader_turn_from_{hold.name}",
        from_index=hold_index,
        to_index=hold_index,
        from_keypoint=hold.name,
        to_keypoint=hold.name,
        frame_id=hold.frame_id,
        vector=(0.0, 0.0, 0.0),
        execution_order=list(step.execution_order),
        primitive="turn_gripper_to_next_keypoint",
        mtc_stage_type="MoveTo",
        planner="CartesianPath",
        executable=executable,
        yaw_delta=yaw_delta,
        target_yaw=target_yaw,
        info=(
            "leader is already aligned at the seat-edge keypoint"
            if not executable
            else "leader rotates in place at the seat-edge keypoint with "
            "zero TCP translation before moving ahead"
        ),
    )


def _seat_cable_spec(
    stage_index: int,
    step: TaskStep,
    keypoints: Sequence[Keypoint],
    follower_group: str,
    follower_ik_frame: str,
    from_index: int,
    to_index: int,
) -> MtcStageSpec:
    start = keypoints[from_index]
    goal = keypoints[to_index]
    return MtcStageSpec(
        step_index=step.index,
        stage_index=stage_index,
        stage_key=_stage_key(
            step, "follower", start, goal, "seat_cable_on_edge"
        ),
        action=step.action,
        actor="follower",
        group=follower_group,
        ik_frame=follower_ik_frame,
        name=f"step_{step.index}_seat_edge_follower_seat_cable",
        from_index=from_index,
        to_index=to_index,
        from_keypoint=start.name,
        to_keypoint=goal.name,
        frame_id=start.frame_id,
        vector=_delta(start, goal),
        execution_order=list(step.execution_order),
        primitive="seat_cable_on_edge",
        mtc_stage_type="InfoOnly",
        planner="none",
        executable=False,
        info=(
            "placeholder for the physical cable-seating action; no arm "
            "motion is issued"
        ),
    )


def _leader_seat_edge_lead_spec(
    stage_index: int,
    step: TaskStep,
    keypoints: Sequence[Keypoint],
    leader_group: str,
    leader_ik_frame: str,
    leader_lead_distance: float,
) -> MtcStageSpec:
    if step.leader_hold_index is None:
        raise ValueError("seat_edge step requires leader_hold_index")
    hold_index = step.leader_hold_index
    hold = keypoints[hold_index]
    lead_vector = _scaled_direction(
        _path_direction(keypoints, hold_index),
        leader_lead_distance,
    )
    return MtcStageSpec(
        step_index=step.index,
        stage_index=stage_index,
        stage_key=(
            f"step_{step.index}:seat_edge:leader:{hold.name}->"
            f"{hold.name}_lead:leader_move_ahead_for_seat_edge"
        ),
        action="seat_edge",
        actor="leader",
        group=leader_group,
        ik_frame=leader_ik_frame,
        name=f"step_{step.index}_seat_edge_leader_move_ahead_from_{hold.name}",
        from_index=hold_index,
        to_index=hold_index,
        from_keypoint=hold.name,
        to_keypoint=f"{hold.name}_lead",
        frame_id=hold.frame_id,
        vector=lead_vector,
        execution_order=list(step.execution_order),
        primitive="leader_move_ahead_for_seat_edge",
        mtc_stage_type="MoveRelative",
        planner="CartesianPath",
        info=(
            "leader moves ahead along the cable path to clear the seat-edge "
            "keypoint before follower motion"
        ),
    )


def build_mtc_stage_specs(
    keypoints: Sequence[Keypoint],
    task_steps: Sequence[TaskStep],
    leader_group: str = "left_fr3_arm",
    follower_group: str = "right_fr3_arm",
    leader_ik_frame: str = "left_fr3_hand_tcp",
    follower_ik_frame: str = "right_fr3_hand_tcp",
    initial_leader_index: int = 1,
    initial_follower_index: int = 0,
    align_initial_poses: bool = True,
    leader_lead_distance: float = DEFAULT_LEADER_LEAD_DISTANCE,
    min_motion_distance: float = 1e-4,
) -> List[MtcStageSpec]:
    if leader_lead_distance <= 0.0:
        raise ValueError("leader_lead_distance must be greater than zero")
    specs: List[MtcStageSpec] = []
    actor_yaw = {
        "leader": (
            _path_yaw(keypoints, initial_leader_index)
            if 0 <= initial_leader_index < len(keypoints)
            else 0.0
        ),
        "follower": (
            _path_yaw(keypoints, initial_follower_index)
            if 0 <= initial_follower_index < len(keypoints)
            else 0.0
        ),
    }
    if len(keypoints) >= 2:
        specs.append(
            _initial_gripper_close_spec(
                len(specs), "follower", follower_group, follower_ik_frame,
                keypoints, initial_follower_index,
            )
        )
        specs.append(
            _initial_gripper_close_spec(
                len(specs), "leader", leader_group, leader_ik_frame,
                keypoints, initial_leader_index,
            )
        )
        if align_initial_poses:
            specs.append(
                _initial_alignment_spec(
                    len(specs),
                    "follower",
                    follower_group,
                    follower_ik_frame,
                    keypoints,
                    initial_follower_index,
                    actor_yaw["follower"],
                )
            )
            specs.append(
                _initial_alignment_spec(
                    len(specs),
                    "leader",
                    leader_group,
                    leader_ik_frame,
                    keypoints,
                    initial_leader_index,
                    actor_yaw["leader"],
                )
            )

    for step in task_steps:
        actor, from_index, to_index = _index_pair_for_step(step)
        if actor == "none" or from_index is None or to_index is None:
            continue

        start = keypoints[from_index]
        goal = keypoints[to_index]
        vector = _delta(start, goal)
        if _norm(vector) < min_motion_distance:
            continue

        group = leader_group if actor == "leader" else follower_group
        ik_frame = leader_ik_frame if actor == "leader" else follower_ik_frame
        base = _base_spec_kwargs(
            step,
            actor,
            group,
            ik_frame,
            start,
            goal,
            from_index,
            to_index,
            vector,
        )

        if step.action == "seat_edge":
            if step.leader_hold_index is None:
                raise ValueError("seat_edge step requires leader_hold_index")
            specs.append(
                _seat_cable_spec(
                    len(specs),
                    step,
                    keypoints,
                    follower_group,
                    follower_ik_frame,
                    from_index,
                    to_index,
                )
            )
            terminal_seat_edge = (
                to_index == len(keypoints) - 1
                and step.leader_hold_index == len(keypoints) - 1
                and from_index == to_index - 1
            )
            if terminal_seat_edge:
                continue
            leader_target_yaw = _path_yaw(keypoints, step.leader_hold_index)
            specs.append(
                _leader_seat_edge_turn_spec(
                    len(specs),
                    step,
                    keypoints,
                    leader_group,
                    leader_ik_frame,
                    actor_yaw["leader"],
                )
            )
            actor_yaw["leader"] = leader_target_yaw
            specs.append(
                _leader_seat_edge_lead_spec(
                    len(specs),
                    step,
                    keypoints,
                    leader_group,
                    leader_ik_frame,
                    leader_lead_distance,
                )
            )

        if step.action in {"straighten", "seat_edge"}:
            target_yaw = _yaw_toward(vector)
            yaw_delta = _normalize_angle(target_yaw - actor_yaw[actor])
            turn_executable = not math.isclose(yaw_delta, 0.0, abs_tol=1e-6)
            specs.append(
                MtcStageSpec(
                    **base,
                    stage_index=len(specs),
                    stage_key=_stage_key(
                        step,
                        actor,
                        start,
                        goal,
                        "turn_gripper_to_next_keypoint",
                    ),
                    name=(
                        f"step_{step.index}_{step.action}_{actor}_"
                        f"turn_to_{goal.name}"
                    ),
                    primitive="turn_gripper_to_next_keypoint",
                    mtc_stage_type="MoveTo",
                    planner="CartesianPath",
                    executable=turn_executable,
                    yaw_delta=yaw_delta,
                    target_yaw=target_yaw,
                    info=(
                        "gripper is already aligned with the next keypoint"
                        if not turn_executable
                        else "rotate gripper in place with zero TCP "
                        "translation to face the next keypoint before moving"
                    ),
                )
            )
            actor_yaw[actor] = target_yaw
            if step.action == "seat_edge":
                goal_yaw = _path_yaw(keypoints, to_index)
                specs.append(
                    MtcStageSpec(
                        **base,
                        stage_index=len(specs),
                        stage_key=_stage_key(
                            step,
                            actor,
                            start,
                            goal,
                            "direct_move_to_seat_edge_keypoint",
                        ),
                        name=(
                            f"step_{step.index}_seat_edge_{actor}_"
                            f"plan_to_{goal.name}"
                        ),
                        primitive="direct_move_to_seat_edge_keypoint",
                        mtc_stage_type="MoveTo",
                        planner="JointInterpolationPlanner",
                        target_yaw=goal_yaw,
                        info=(
                            "follower plans in joint space to the seat-edge "
                            "keypoint and faces along the outgoing cable path"
                        ),
                    )
                )
                actor_yaw[actor] = goal_yaw
                continue

            specs.append(
                MtcStageSpec(
                    **base,
                    stage_index=len(specs),
                    stage_key=_stage_key(
                        step,
                        actor,
                        start,
                        goal,
                        "cartesian_move_to_next_keypoint",
                    ),
                    name=(
                        f"step_{step.index}_{step.action}_{actor}_"
                        f"cartesian_{start.name}_to_{goal.name}"
                    ),
                    primitive="cartesian_move_to_next_keypoint",
                    mtc_stage_type="MoveRelative",
                    planner="CartesianPath",
                    info="cartesian move to next keypoint with gripper closed",
                )
            )
            continue

        if step.action == "move_anchor":
            goal_yaw = _path_yaw(keypoints, to_index)
            specs.append(
                MtcStageSpec(
                    **base,
                    stage_index=len(specs),
                    stage_key=_stage_key(
                        step,
                        actor,
                        start,
                        goal,
                        "direct_move_to_next_anchor",
                    ),
                    name=(
                        f"step_{step.index}_move_anchor_{actor}_"
                        f"plan_to_{goal.name}"
                    ),
                    primitive="direct_move_to_next_anchor",
                    mtc_stage_type="MoveTo",
                    planner="JointInterpolationPlanner",
                    target_yaw=goal_yaw,
                    info=(
                        "leader directly plans to the next anchor pose "
                        "with gripper closed and faces along the cable path"
                    ),
                )
            )
            actor_yaw[actor] = goal_yaw
            continue

        specs.append(
            MtcStageSpec(
                **base,
                stage_index=len(specs),
                stage_key=_stage_key(
                    step,
                    actor,
                    start,
                    goal,
                    "unknown_action_info_only",
                ),
                name=f"step_{step.index}_{step.action}_info_only",
                primitive="unknown_action_info_only",
                mtc_stage_type="InfoOnly",
                planner="none",
                executable=False,
                info=f"action {step.action} is not implemented",
            )
        )
    return specs


def mtc_stage_spec_to_dict(spec: MtcStageSpec) -> Dict[str, Any]:
    return {
        "stage_index": spec.stage_index,
        "stage_key": spec.stage_key,
        "task_step_index": spec.step_index,
        "action": spec.action,
        "actor": spec.actor,
        "group": spec.group,
        "ik_frame": spec.ik_frame,
        "stage_name": spec.name,
        "primitive": spec.primitive,
        "from_index": spec.from_index,
        "to_index": spec.to_index,
        "from": spec.from_keypoint,
        "to": spec.to_keypoint,
        "frame_id": spec.frame_id,
        "relative_vector": list(spec.vector),
        "yaw_delta": spec.yaw_delta,
        "target_yaw": spec.target_yaw,
        "joint_goal": dict(spec.joint_goal),
        "execution_order": list(spec.execution_order),
        "mtc_stage_type": spec.mtc_stage_type,
        "planner": spec.planner,
        "executable": spec.executable,
        "info": spec.info,
    }


def mtc_stage_sequence_to_dict(
    specs: Sequence[MtcStageSpec],
    task_name: str = "dual_fr3_trunking_mtc_prototype",
) -> Dict[str, Any]:
    return {
        "interface_version": 1,
        "task_name": task_name,
        "stage_count": len(specs),
        "stage_sequence": [
            mtc_stage_spec_to_dict(spec)
            for spec in specs
        ],
    }


def mtc_stage_sequence_to_json(specs: Sequence[MtcStageSpec]) -> str:
    return json.dumps(
        mtc_stage_sequence_to_dict(specs),
        ensure_ascii=False,
        indent=2,
    )


def mtc_stage_sequence_to_text(specs: Sequence[MtcStageSpec]) -> str:
    lines = []
    for spec in specs:
        execution_order = " -> ".join(spec.execution_order)
        lines.append(
            f"{spec.stage_index}: stage_key={spec.stage_key}, "
            f"task_step={spec.step_index}, "
            f"action={spec.action}, actor={spec.actor}, "
            f"primitive={spec.primitive}, type={spec.mtc_stage_type}, "
            f"group={spec.group}, ik_frame={spec.ik_frame}, "
            f"{spec.from_keypoint}->{spec.to_keypoint}, "
            f"frame={spec.frame_id}, vector={spec.vector}, "
            f"yaw_delta={spec.yaw_delta:.3f}, "
            f"target_yaw={spec.target_yaw:.3f}, "
            f"executable={spec.executable}, "
            f"order={execution_order}, info={spec.info}"
        )
    return "\n".join(lines)


def _import_mtc_modules():
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


def create_mtc_task(
    node,
    keypoints: Sequence[Keypoint],
    task_steps: Sequence[TaskStep],
    leader_group: str = "left_fr3_arm",
    follower_group: str = "right_fr3_arm",
    leader_ik_frame: str = "left_fr3_hand_tcp",
    follower_ik_frame: str = "right_fr3_hand_tcp",
    cartesian_step_size: float = 0.01,
    motion_velocity_scaling: float = 0.2,
    motion_acceleration_scaling: float = 0.2,
    initial_leader_index: int = 1,
    initial_follower_index: int = 0,
    align_initial_poses: bool = True,
    leader_lead_distance: float = DEFAULT_LEADER_LEAD_DISTANCE,
    tool_roll: float = DEFAULT_TOOL_ROLL,
    tool_pitch: float = DEFAULT_TOOL_PITCH,
    selected_stage_indices: set[int] | None = None,
):
    _rclcpp, core, stages = _import_mtc_modules()

    cartesian = core.CartesianPath()
    cartesian.step_size = cartesian_step_size
    cartesian.jump_threshold = 0.0
    cartesian.max_velocity_scaling_factor = motion_velocity_scaling
    cartesian.max_acceleration_scaling_factor = motion_acceleration_scaling
    jointspace = core.JointInterpolationPlanner()
    jointspace.max_velocity_scaling_factor = motion_velocity_scaling
    jointspace.max_acceleration_scaling_factor = motion_acceleration_scaling

    task = core.Task()
    task.name = "dual_fr3_trunking_mtc_prototype"
    task.loadRobotModel(node)
    task.add(stages.CurrentState("current_state"))

    specs = build_mtc_stage_specs(
        keypoints,
        task_steps,
        leader_group=leader_group,
        follower_group=follower_group,
        leader_ik_frame=leader_ik_frame,
        follower_ik_frame=follower_ik_frame,
        initial_leader_index=initial_leader_index,
        initial_follower_index=initial_follower_index,
        align_initial_poses=align_initial_poses,
        leader_lead_distance=leader_lead_distance,
    )
    for spec in specs:
        if (
            selected_stage_indices is not None
            and spec.stage_index not in selected_stage_indices
        ):
            continue
        if not spec.executable:
            continue

        if spec.mtc_stage_type == "MoveTo":
            planner = (
                cartesian
                if spec.primitive == "turn_gripper_to_next_keypoint"
                else jointspace
            )
            move_to = stages.MoveTo(spec.name, planner)
            move_to.group = spec.group
            if spec.primitive == "turn_gripper_to_next_keypoint":
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
                "move_to_initial_keypoint",
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


def _default_keypoints_file() -> str:
    return str(
        Path(get_package_share_directory("dual_fr3_trunking_mtc"))
        / "config"
        / "keypoints.yaml"
    )


def _parse_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    return value.lower() in {"1", "true", "yes", "on"}


def _parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--keypoints-file", default=_default_keypoints_file())
    parser.add_argument("--task-frame", default="left_fr3_link0")
    parser.add_argument("--initial-leader-index", type=int, default=1)
    parser.add_argument("--initial-follower-index", type=int, default=0)
    parser.add_argument("--leader-group", default="left_fr3_arm")
    parser.add_argument("--follower-group", default="right_fr3_arm")
    parser.add_argument("--leader-ik-frame", default="left_fr3_hand_tcp")
    parser.add_argument("--follower-ik-frame", default="right_fr3_hand_tcp")
    parser.add_argument("--cartesian-step-size", type=float, default=0.01)
    parser.add_argument("--motion-velocity-scaling", type=float, default=0.2)
    parser.add_argument("--motion-acceleration-scaling", type=float, default=0.2)
    parser.add_argument(
        "--leader-lead-distance",
        type=float,
        default=DEFAULT_LEADER_LEAD_DISTANCE,
    )
    parser.add_argument("--tool-roll", type=float, default=DEFAULT_TOOL_ROLL)
    parser.add_argument("--tool-pitch", type=float, default=DEFAULT_TOOL_PITCH)
    parser.add_argument("--align-initial-poses", type=_parse_bool, default=True)
    parser.add_argument("--plan", type=_parse_bool, default=True)
    parser.add_argument("--execute", type=_parse_bool, default=False)
    parser.add_argument(
        "--execute-stage-by-stage",
        type=_parse_bool,
        default=True,
        help=(
            "replan from the real robot state before each executable stage; "
            "disable only for legacy whole-solution execution"
        ),
    )
    parser.add_argument("--publish-solution", type=_parse_bool, default=True)
    parser.add_argument("--keep-alive-sec", type=float, default=2.0)
    args, _unknown = parser.parse_known_args(list(argv))
    return args


def _start_stage_sequence_publisher(
    specs: Sequence[MtcStageSpec],
    logger: logging.Logger,
):
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
    except ImportError as exc:
        logger.warning("rclpy is not importable; stage sequence topic skipped: %s", exc)
        return None

    initialized_here = False
    interface_node = None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True

        interface_node = rclpy.create_node(
            "dual_fr3_trunking_mtc_prototype_interface"
        )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        json_pub = interface_node.create_publisher(
            String,
            "/dual_fr3_trunking_mtc_prototype/stage_sequence",
            latched_qos,
        )
        text_pub = interface_node.create_publisher(
            String,
            "/dual_fr3_trunking_mtc_prototype/stage_sequence_text",
            latched_qos,
        )

        json_msg = String()
        json_msg.data = mtc_stage_sequence_to_json(specs)
        text_msg = String()
        text_msg.data = mtc_stage_sequence_to_text(specs)
        json_pub.publish(json_msg)
        text_pub.publish(text_msg)
        rclpy.spin_once(interface_node, timeout_sec=0.1)
        logger.info(
            "published MTC stage sequence to "
            "/dual_fr3_trunking_mtc_prototype/stage_sequence"
        )
        return (rclpy, interface_node, initialized_here)
    except Exception as exc:
        logger.warning("stage sequence topic publish skipped: %s", exc)
        if interface_node is not None:
            interface_node.destroy_node()
        if initialized_here:
            rclpy.shutdown()
        return None


def _spin_stage_sequence_publisher(handle, timeout_sec: float = 0.1) -> None:
    if handle is None:
        return
    rclpy, interface_node, _initialized_here = handle
    rclpy.spin_once(interface_node, timeout_sec=timeout_sec)


def _shutdown_stage_sequence_publisher(handle) -> None:
    if handle is None:
        return
    rclpy, interface_node, initialized_here = handle
    interface_node.destroy_node()
    if initialized_here:
        rclpy.shutdown()


def _execute_stage_by_stage(
    node,
    specs: Sequence[MtcStageSpec],
    keypoints: Sequence[Keypoint],
    task_steps: Sequence[TaskStep],
    args: argparse.Namespace,
    logger: logging.Logger,
) -> bool:
    executable_specs = [spec for spec in specs if spec.executable]
    logger.info(
        "executing %d MTC stages with real-state replan between stages",
        len(executable_specs),
    )
    for ordinal, spec in enumerate(executable_specs, start=1):
        logger.info(
            "planning stage %d/%d: [%02d] %s",
            ordinal,
            len(executable_specs),
            spec.stage_index,
            spec.name,
        )
        stage_task, _ = create_mtc_task(
            node,
            keypoints,
            task_steps,
            leader_group=args.leader_group,
            follower_group=args.follower_group,
            leader_ik_frame=args.leader_ik_frame,
            follower_ik_frame=args.follower_ik_frame,
            cartesian_step_size=args.cartesian_step_size,
            motion_velocity_scaling=args.motion_velocity_scaling,
            motion_acceleration_scaling=args.motion_acceleration_scaling,
            initial_leader_index=args.initial_leader_index,
            initial_follower_index=args.initial_follower_index,
            align_initial_poses=args.align_initial_poses,
            leader_lead_distance=args.leader_lead_distance,
            tool_roll=args.tool_roll,
            tool_pitch=args.tool_pitch,
            selected_stage_indices={spec.stage_index},
        )
        if not stage_task.plan() or not stage_task.solutions:
            logger.error(
                "planning failed for stage [%02d] %s",
                spec.stage_index,
                spec.name,
            )
            return False
        result = stage_task.execute(stage_task.solutions[0])
        if not result:
            logger.error(
                "execution failed for stage [%02d] %s (MoveIt error %s)",
                spec.stage_index,
                spec.name,
                getattr(result, "val", result),
            )
            return False
        logger.info("stage [%02d] completed", spec.stage_index)
    return True


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    logger = _make_logger()
    rclcpp, _core, _stages = _import_mtc_modules()

    rclcpp.init()
    node = rclcpp.Node("dual_fr3_trunking_mtc_prototype")
    stage_publisher = None
    try:
        keypoints = load_keypoints(args.keypoints_file, fallback_frame=args.task_frame)
        segments = build_segment_plans(keypoints)
        task_steps = build_task_schedule(
            keypoints,
            initial_leader_index=args.initial_leader_index,
            initial_follower_index=args.initial_follower_index,
        )
        task, specs = create_mtc_task(
            node,
            keypoints,
            task_steps,
            leader_group=args.leader_group,
            follower_group=args.follower_group,
            leader_ik_frame=args.leader_ik_frame,
            follower_ik_frame=args.follower_ik_frame,
            cartesian_step_size=args.cartesian_step_size,
            motion_velocity_scaling=args.motion_velocity_scaling,
            motion_acceleration_scaling=args.motion_acceleration_scaling,
            initial_leader_index=args.initial_leader_index,
            initial_follower_index=args.initial_follower_index,
            align_initial_poses=args.align_initial_poses,
            leader_lead_distance=args.leader_lead_distance,
            tool_roll=args.tool_roll,
            tool_pitch=args.tool_pitch,
        )

        logger.info(
            "loaded %d keypoints, %d segments, %d task steps, %d MTC stages",
            len(keypoints),
            len(segments),
            len(task_steps),
            len(specs),
        )
        logger.info("MTC stage sequence:\n%s", mtc_stage_sequence_to_text(specs))
        if args.plan:
            full_plan_succeeded = task.plan()
            if full_plan_succeeded:
                logger.info(
                    "planning succeeded with %d solutions",
                    len(task.solutions),
                )
                if args.publish_solution and task.solutions:
                    task.publish(task.solutions[0])
            elif not (args.execute and args.execute_stage_by_stage):
                logger.error("planning failed")
                return 2
            else:
                logger.warning(
                    "whole-chain planning failed; trying staged execution "
                    "from fresh robot states"
                )

            if args.execute and args.execute_stage_by_stage:
                if not _execute_stage_by_stage(
                    node,
                    specs,
                    keypoints,
                    task_steps,
                    args,
                    logger,
                ):
                    return 3
                logger.info("staged MTC execution finished")
            elif args.execute and full_plan_succeeded and task.solutions:
                logger.warning(
                    "executing the whole MTC solution without real-state "
                    "replanning; this is legacy collision-prone behavior"
                )
                result = task.execute(task.solutions[0])
                if not result:
                    logger.error(
                        "whole-solution MTC execution failed (MoveIt error %s)",
                        getattr(result, "val", result),
                    )
                    return 3
                logger.info("MTC execution request finished")
        else:
            logger.info("build-only mode; planning skipped")

        stage_publisher = _start_stage_sequence_publisher(specs, logger)
        if args.keep_alive_sec > 0.0:
            deadline = time.monotonic() + args.keep_alive_sec
            while time.monotonic() < deadline:
                _spin_stage_sequence_publisher(stage_publisher, timeout_sec=0.1)
        return 0
    finally:
        _shutdown_stage_sequence_publisher(stage_publisher)
        rclcpp.shutdown()
