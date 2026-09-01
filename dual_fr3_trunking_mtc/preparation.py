from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Sequence

from geometry_msgs.msg import Pose, PoseStamped, Vector3, Vector3Stamped
from std_msgs.msg import Header

from .models import (
    DEFAULT_FOLLOWER_ORIENTATION_DIRECTION,
    DEFAULT_LEADER_ORIENTATION_DIRECTION,
    DEFAULT_TOOL_PITCH,
    DEFAULT_TOOL_ROLL,
    Keypoint,
    path_orientation_yaw,
    rpy_to_quaternion,
    validate_orientation_direction,
)


LEADER_APPROACH = "leader_move_above_initial_keypoint"
FOLLOWER_APPROACH = "follower_move_above_initial_keypoint"
LEADER_CLOSE = "leader_close_gripper"
FOLLOWER_CLOSE = "follower_close_gripper"
DUAL_DESCENT = "dual_cartesian_descent"
TERMINAL_DEVICE = "/dev/tty"

PREPARATION_STEP_KEYS = (
    LEADER_APPROACH,
    FOLLOWER_APPROACH,
    LEADER_CLOSE,
    FOLLOWER_CLOSE,
    DUAL_DESCENT,
)


@dataclass(frozen=True)
class PreparationConfig:
    leader_index: int = 1
    follower_index: int = 0
    approach_height: float = 0.15
    leader_group: str = "left_fr3_arm"
    follower_group: str = "right_fr3_arm"
    leader_ik_frame: str = "left_fr3_hand_tcp"
    follower_ik_frame: str = "right_fr3_hand_tcp"
    leader_orientation_direction: str = DEFAULT_LEADER_ORIENTATION_DIRECTION
    follower_orientation_direction: str = DEFAULT_FOLLOWER_ORIENTATION_DIRECTION
    tool_roll: float = DEFAULT_TOOL_ROLL
    tool_pitch: float = DEFAULT_TOOL_PITCH
    cartesian_step_size: float = 0.01
    velocity_scaling: float = 0.2
    acceleration_scaling: float = 0.2
    ompl_pipeline: str = "move_group"
    ompl_planner_id: str = "RRTConnectkConfigDefault"
    ompl_planning_attempts: int = 5
    ompl_timeout: float = 5.0
    interactive: bool = True


@dataclass(frozen=True)
class PreparationStep:
    key: str
    description: str
    confirmation_required: bool = False


def _validate_config(
    keypoints: Sequence[Keypoint],
    config: PreparationConfig,
) -> None:
    if not keypoints:
        raise ValueError("preparation requires at least one keypoint")
    for label, index in (
        ("leader_index", config.leader_index),
        ("follower_index", config.follower_index),
    ):
        if not 0 <= index < len(keypoints):
            raise ValueError(f"{label} out of range: {index}")
    if not math.isfinite(config.approach_height) or config.approach_height <= 0.0:
        raise ValueError("preparation approach_height must be finite and positive")
    validate_orientation_direction(config.leader_orientation_direction)
    validate_orientation_direction(config.follower_orientation_direction)


def build_preparation_steps(
    keypoints: Sequence[Keypoint],
    config: PreparationConfig,
) -> list[PreparationStep]:
    _validate_config(keypoints, config)
    leader_goal = keypoints[config.leader_index]
    follower_goal = keypoints[config.follower_index]
    height = config.approach_height
    return [
        PreparationStep(
            LEADER_APPROACH,
            f"move leader above {leader_goal.name} by {height:.3f} m",
        ),
        PreparationStep(
            FOLLOWER_APPROACH,
            f"move follower above {follower_goal.name} by {height:.3f} m",
        ),
        PreparationStep(
            LEADER_CLOSE,
            "close leader gripper",
            confirmation_required=True,
        ),
        PreparationStep(
            FOLLOWER_CLOSE,
            "close follower gripper",
            confirmation_required=True,
        ),
        PreparationStep(
            DUAL_DESCENT,
            f"move both TCPs down {height:.3f} m on synchronized Cartesian paths",
            confirmation_required=True,
        ),
    ]


def preparation_sequence_to_text(
    keypoints: Sequence[Keypoint],
    config: PreparationConfig,
) -> str:
    return "\n".join(
        f"[{index:02d}] {step.key}: {step.description}"
        for index, step in enumerate(build_preparation_steps(keypoints, config))
    )


def _path_yaw(
    keypoints: Sequence[Keypoint],
    index: int,
    orientation_direction: str,
) -> float:
    if len(keypoints) < 2:
        raise ValueError("at least two keypoints are required to derive preparation yaw")
    if index < len(keypoints) - 1:
        start = keypoints[index]
        goal = keypoints[index + 1]
    else:
        start = keypoints[index - 1]
        goal = keypoints[index]
    if start.frame_id != goal.frame_id:
        raise ValueError(
            "cannot derive preparation yaw across different keypoint frames: "
            f"{start.frame_id} != {goal.frame_id}"
        )
    return path_orientation_yaw(
        goal.position[0] - start.position[0],
        goal.position[1] - start.position[1],
        orientation_direction,
    )


def _identity_ik_frame(frame_id: str) -> PoseStamped:
    frame = PoseStamped(header=Header(frame_id=frame_id), pose=Pose())
    frame.pose.orientation.w = 1.0
    return frame


def _approach_pose(
    keypoints: Sequence[Keypoint],
    index: int,
    height: float,
    tool_roll: float,
    tool_pitch: float,
    orientation_direction: str,
) -> PoseStamped:
    keypoint = keypoints[index]
    pose = PoseStamped()
    pose.header.frame_id = keypoint.frame_id
    pose.pose.position.x = keypoint.position[0]
    pose.pose.position.y = keypoint.position[1]
    pose.pose.position.z = keypoint.position[2] + height
    pose.pose.orientation = rpy_to_quaternion(
        tool_roll,
        tool_pitch,
        _path_yaw(keypoints, index, orientation_direction),
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


def _create_planners(core, node, config: PreparationConfig):
    cartesian = core.CartesianPath()
    cartesian.step_size = config.cartesian_step_size
    cartesian.jump_threshold = 0.0
    cartesian.max_velocity_scaling_factor = config.velocity_scaling
    cartesian.max_acceleration_scaling_factor = config.acceleration_scaling

    jointspace = core.JointInterpolationPlanner()
    jointspace.max_velocity_scaling_factor = config.velocity_scaling
    jointspace.max_acceleration_scaling_factor = config.acceleration_scaling

    ompl = core.PipelinePlanner(node, config.ompl_pipeline)
    ompl.planner = config.ompl_planner_id
    ompl.num_planning_attempts = config.ompl_planning_attempts
    ompl.max_velocity_scaling_factor = config.velocity_scaling
    ompl.max_acceleration_scaling_factor = config.acceleration_scaling
    return cartesian, jointspace, ompl


def _approach_stage(
    stages,
    planner,
    name: str,
    group: str,
    ik_frame: str,
    goal: PoseStamped,
    timeout: float,
):
    move = stages.MoveTo(name, planner)
    move.timeout = timeout
    move.group = group
    move.ik_frame = _identity_ik_frame(ik_frame)
    move.setGoal(goal)
    return move


def _gripper_close_stage(
    stages,
    planner,
    name: str,
    arm_group: str,
    ik_frame: str,
):
    move = stages.MoveTo(name, planner)
    move.group = _hand_group_for_arm_group(arm_group)
    move.setGoal({_finger_joint_for_ik_frame(ik_frame): 0.0})
    return move


def _descent_stage(
    core,
    stages,
    planner,
    keypoints: Sequence[Keypoint],
    config: PreparationConfig,
):
    merger = core.Merger("preparation_dual_cartesian_descent")
    for actor, group, ik_frame, index in (
        (
            "leader",
            config.leader_group,
            config.leader_ik_frame,
            config.leader_index,
        ),
        (
            "follower",
            config.follower_group,
            config.follower_ik_frame,
            config.follower_index,
        ),
    ):
        move = stages.MoveRelative(
            f"preparation_{actor}_cartesian_descent",
            planner,
        )
        move.group = group
        move.ik_frame = _identity_ik_frame(ik_frame)
        move.setDirection(
            Vector3Stamped(
                header=Header(frame_id=keypoints[index].frame_id),
                vector=Vector3(z=-config.approach_height),
            )
        )
        merger.insert(move)
    return merger


def add_preparation_stages(
    task,
    core,
    stages,
    node,
    keypoints: Sequence[Keypoint],
    config: PreparationConfig,
    selected_step_keys: set[str] | None = None,
) -> list[PreparationStep]:
    steps = build_preparation_steps(keypoints, config)
    known_keys = {step.key for step in steps}
    selected = known_keys if selected_step_keys is None else selected_step_keys
    unknown = selected - known_keys
    if unknown:
        raise ValueError(f"unknown preparation step(s): {sorted(unknown)}")

    cartesian, jointspace, ompl = _create_planners(core, node, config)
    for step in steps:
        if step.key not in selected:
            continue
        if step.key == LEADER_APPROACH:
            task.add(
                _approach_stage(
                    stages,
                    ompl,
                    "preparation_leader_move_above_initial_keypoint",
                    config.leader_group,
                    config.leader_ik_frame,
                    _approach_pose(
                        keypoints,
                        config.leader_index,
                        config.approach_height,
                        config.tool_roll,
                        config.tool_pitch,
                        config.leader_orientation_direction,
                    ),
                    config.ompl_timeout,
                )
            )
        elif step.key == FOLLOWER_APPROACH:
            task.add(
                _approach_stage(
                    stages,
                    ompl,
                    "preparation_follower_move_above_initial_keypoint",
                    config.follower_group,
                    config.follower_ik_frame,
                    _approach_pose(
                        keypoints,
                        config.follower_index,
                        config.approach_height,
                        config.tool_roll,
                        config.tool_pitch,
                        config.follower_orientation_direction,
                    ),
                    config.ompl_timeout,
                )
            )
        elif step.key == LEADER_CLOSE:
            task.add(
                _gripper_close_stage(
                    stages,
                    jointspace,
                    "preparation_leader_close_gripper",
                    config.leader_group,
                    config.leader_ik_frame,
                )
            )
        elif step.key == FOLLOWER_CLOSE:
            task.add(
                _gripper_close_stage(
                    stages,
                    jointspace,
                    "preparation_follower_close_gripper",
                    config.follower_group,
                    config.follower_ik_frame,
                )
            )
        elif step.key == DUAL_DESCENT:
            task.add(_descent_stage(core, stages, cartesian, keypoints, config))
    return steps


def create_preparation_task(
    node,
    core,
    stages,
    keypoints: Sequence[Keypoint],
    config: PreparationConfig,
    selected_step_keys: set[str] | None = None,
):
    task = core.Task()
    task.name = "dual_fr3_trunking_preparation"
    task.loadRobotModel(node)
    task.add(stages.CurrentState("preparation_current_state"))
    steps = add_preparation_stages(
        task,
        core,
        stages,
        node,
        keypoints,
        config,
        selected_step_keys=selected_step_keys,
    )
    return task, steps


def _confirm_step(
    step: PreparationStep,
    input_fn: Callable[[str], str],
    logger,
) -> bool:
    # ros2 launch prefixes and forwards complete output lines.  A normal
    # input() prompt has no trailing newline, so it can remain buffered and
    # make an intentional operator wait look like a hang.  Log the complete
    # instruction first, then read an unprompted line from the same terminal.
    logger.warning(
        "WAITING FOR KEYBOARD CONFIRMATION: %s. "
        "Press Enter to execute, or type q and press Enter to abort.",
        step.description,
    )
    while True:
        try:
            answer = input_fn("").strip().lower()
        except (EOFError, OSError) as error:
            logger.error(
                "keyboard input is unavailable; aborting interactive preparation: %s",
                error,
            )
            return False
        if answer in {"", "y", "yes"}:
            return True
        if answer in {"q", "quit", "n", "no"}:
            logger.warning("preparation aborted by operator before %s", step.key)
            return False
        logger.warning("unrecognized input %r; press Enter or q", answer)


def _read_controlling_terminal(_prompt: str) -> str:
    """Read a command from the terminal that owns the ros2 launch process.

    ROS 2 launch does not forward its stdin to launched Node processes.  Opening
    /dev/tty bypasses the detached child stdin while keeping confirmation in the
    same terminal from which the operator started ros2 launch.
    """
    # Open read-only: a text stream opened as r+ may require seeking between
    # reads and writes, while terminal devices are intentionally non-seekable.
    with open(TERMINAL_DEVICE, "r", encoding="utf-8", buffering=1) as terminal:
        answer = terminal.readline()
    if answer == "":
        raise EOFError("controlling terminal was closed")
    return answer


def run_preparation(
    node,
    core,
    stages,
    keypoints: Sequence[Keypoint],
    config: PreparationConfig,
    logger,
    input_fn: Callable[[str], str] = _read_controlling_terminal,
) -> bool:
    steps = build_preparation_steps(keypoints, config)
    for index, step in enumerate(steps, start=1):
        if config.interactive and step.confirmation_required:
            if not _confirm_step(step, input_fn, logger):
                return False

        logger.info(
            "planning preparation step %d/%d: %s",
            index,
            len(steps),
            step.description,
        )
        try:
            task, _ = create_preparation_task(
                node,
                core,
                stages,
                keypoints,
                config,
                selected_step_keys={step.key},
            )
            if not task.plan() or not task.solutions:
                logger.error("preparation planning failed at %s", step.key)
                return False
            result = task.execute(task.solutions[0])
        except Exception:  # noqa: BLE001 - preparation must fail closed
            logger.exception("preparation raised at %s", step.key)
            return False
        if not result:
            logger.error(
                "preparation execution failed at %s (MoveIt error %s)",
                step.key,
                getattr(result, "val", result),
            )
            return False
        logger.info("preparation step completed: %s", step.key)
    return True
