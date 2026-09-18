from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Sequence

from dual_fr3_trunking_mtc.task.models import (
    Keypoint,
    TaskPlan,
    path_orientation_yaw,
    validate_orientation_direction,
    rpy_to_quaternion,
)
from dual_fr3_trunking_mtc.runtime.config import DEFAULTS

if TYPE_CHECKING:
    from dual_fr3_trunking_mtc.stages.specs import MtcStageSpec


LEADER_APPROACH = "leader_move_above_initial_keypoint"
FOLLOWER_APPROACH = "follower_move_above_initial_keypoint"
LEADER_CLOSE = "leader_close_gripper"
FOLLOWER_CLOSE = "follower_close_gripper"
INSERT_CABLE = "insert_simulation_cable"
VERIFY_USB = "release_and_verify_usb_grasp"
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
    approach_height: float = DEFAULTS.preparation_height
    leader_group: str = DEFAULTS.leader_group
    follower_group: str = DEFAULTS.follower_group
    leader_ik_frame: str = DEFAULTS.leader_ik_frame
    follower_ik_frame: str = DEFAULTS.follower_ik_frame
    leader_orientation_direction: str = DEFAULTS.leader_orientation_direction
    follower_orientation_direction: str = DEFAULTS.follower_orientation_direction
    interactive: bool = DEFAULTS.preparation_interactive
    leader_gripper_profile: str = DEFAULTS.preparation_leader_gripper_profile
    follower_gripper_profile: str = DEFAULTS.preparation_follower_gripper_profile
    simulation_cable_config: str = ""
    load_cable: bool = True
    tool_roll: float = DEFAULTS.tool_roll
    tool_pitch: float = DEFAULTS.tool_pitch


def _validate_config(
    task_plan: TaskPlan,
    config: PreparationConfig,
) -> None:
    keypoints = task_plan.keypoints
    if not keypoints:
        raise ValueError("preparation requires at least one keypoint")
    for label, index in (
        ("initial_leader_index", task_plan.initial_leader_index),
        ("initial_follower_index", task_plan.initial_follower_index),
    ):
        if not 0 <= index < len(keypoints):
            raise ValueError(f"{label} out of range: {index}")
    if not math.isfinite(config.approach_height) or config.approach_height <= 0.0:
        raise ValueError("preparation approach_height must be finite and positive")
    validate_orientation_direction(config.leader_orientation_direction)
    validate_orientation_direction(config.follower_orientation_direction)
    if config.simulation_cable_config and (
        config.leader_group != "left_fr3_arm" or config.follower_group != "right_fr3_arm"
        or config.leader_ik_frame != "left_fr3_hand_tcp"
        or config.follower_ik_frame != "right_fr3_hand_tcp"
    ):
        raise ValueError("Simulation cable requires leader=left FR3 TCP and follower=right FR3 TCP")


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


def _hand_group_for_arm_group(arm_group: str) -> str:
    if arm_group.endswith("_arm"):
        return f"{arm_group[:-4]}_hand"
    return f"{arm_group}_hand"


def build_preparation_stage_specs(
    task_plan: TaskPlan,
    config: PreparationConfig,
    start_stage_index: int = 0,
) -> list[MtcStageSpec]:
    """Compile preparation into the same stage-spec IR as the formal task."""
    from dual_fr3_trunking_mtc.stages.specs import MtcStageSpec

    _validate_config(task_plan, config)
    keypoints = task_plan.keypoints
    height = config.approach_height
    cable_widths = {}
    if config.simulation_cable_config:
        from dual_fr3_maniskill.cable.model import load_geometry_config
        usb = load_geometry_config(config.simulation_cable_config)["usb"]
        # A force-limited target beyond contact closes against the actual body.
        # Stalling ends this command; release/verification decides grasp success.
        cable_widths = {"leader": 0., "follower": 0.}
        open_width = 2*usb.get("open_finger_position", .02)
    actor_settings = {
        "leader": (
            task_plan.initial_leader_index,
            config.leader_group,
            config.leader_ik_frame,
            config.leader_orientation_direction,
            config.leader_gripper_profile,
        ),
        "follower": (
            task_plan.initial_follower_index,
            config.follower_group,
            config.follower_ik_frame,
            config.follower_orientation_direction,
            config.follower_gripper_profile,
        ),
    }
    specs: list[MtcStageSpec] = []

    actors = ("leader", "follower")
    preparation_poses = {}
    for actor in actors:
        index, _group, _tcp, direction, _profile = actor_settings[actor]
        point = keypoints[index]
        q = rpy_to_quaternion(config.tool_roll, config.tool_pitch, _path_yaw(keypoints, index, direction))
        preparation_poses["left" if actor == "leader" else "right"] = dict(
            frame=point.frame_id, position_m=[*point.position[:2], point.position[2]+height],
            quaternion_wxyz=[q.w, q.x, q.y, q.z], keypoint=point.name)

    def simulation_operation(operation, key, info):
        specs.append(MtcStageSpec(
            step_index=len(specs), stage_index=start_stage_index + len(specs),
            stage_key=f"preparation:usb:{key}", action="preparation",
            actor="leader", group="", ik_frame=config.leader_ik_frame,
            name=f"preparation_{key}", from_index=-1, to_index=-1,
            from_keypoint="prepared_usb", to_keypoint=operation,
            frame_id="world", vector=(0., 0., 0.), execution_order=[key],
            primitive=key, mtc_stage_type="SimulationCable", planner="ModifyPlanningScene",
            phase="preparation", cable_config=config.simulation_cable_config,
            cable_operation=operation,
            cable_orientation_direction=config.leader_orientation_direction, info=info,
            cable_preparation_poses=preparation_poses,
        ))

    if config.simulation_cable_config:
        for actor in actors:
            index, group, ik_frame, _direction, _profile = actor_settings[actor]
            keypoint = keypoints[index]
            key = actor + "_open_gripper"
            specs.append(MtcStageSpec(
                step_index=len(specs), stage_index=start_stage_index + len(specs),
                stage_key=f"preparation:{actor}:{key}", action="preparation", actor=actor,
                group=_hand_group_for_arm_group(group), ik_frame=ik_frame,
                name=f"preparation_{key}", from_index=index, to_index=index,
                from_keypoint=keypoint.name, to_keypoint=keypoint.name, frame_id=keypoint.frame_id,
                vector=(0., 0., 0.), execution_order=[key], primitive="gripper_operation",
                mtc_stage_type="GripperOperation", planner="GripperProfile", gripper_profile="open",
                gripper_action="move", gripper_width_override=open_width,
                phase="preparation", info=f"open {actor} gripper before approaching USB",
            ))

    if config.simulation_cable_config:
        simulation_operation("spawn", INSERT_CABLE,
            "position USB and cable at keypoint preparation targets before either arm approaches")

    for actor, key in (
        ("leader", LEADER_APPROACH),
        ("follower", FOLLOWER_APPROACH),
    ):
        if actor not in actors:
            continue
        index, group, ik_frame, direction, _profile = actor_settings[actor]
        keypoint = keypoints[index]
        specs.append(
            MtcStageSpec(
                step_index=len(specs),
                stage_index=start_stage_index + len(specs),
                stage_key=f"preparation:{actor}:{keypoint.name}:{key}",
                action="preparation",
                actor=actor,
                group=group,
                ik_frame=ik_frame,
                name=f"preparation_{key}",
                from_index=index,
                to_index=index,
                from_keypoint=keypoint.name,
                to_keypoint=f"{keypoint.name}_above",
                frame_id=keypoint.frame_id,
                vector=(0.0, 0.0, height),
                execution_order=[key],
                primitive="move_above_initial_keypoint",
                mtc_stage_type="MoveTo",
                planner="PipelinePlanner",
                target_yaw=_path_yaw(keypoints, index, direction),
                phase="preparation",
                info=f"move {actor} above {keypoint.name} by {height:.3f} m",
            )
        )

    for actor, key in (
        ("leader", LEADER_CLOSE),
        ("follower", FOLLOWER_CLOSE),
    ):
        if actor not in actors:
            continue
        index, group, ik_frame, _direction, profile = actor_settings[actor]
        keypoint = keypoints[index]
        specs.append(
            MtcStageSpec(
                step_index=len(specs),
                stage_index=start_stage_index + len(specs),
                stage_key=f"preparation:{actor}:{keypoint.name}:{key}",
                action="preparation",
                actor=actor,
                group=_hand_group_for_arm_group(group),
                ik_frame=ik_frame,
                name=f"preparation_{key}",
                from_index=index,
                to_index=index,
                from_keypoint=keypoint.name,
                to_keypoint=keypoint.name,
                frame_id=keypoint.frame_id,
                vector=(0.0, 0.0, 0.0),
                execution_order=[key],
                primitive="gripper_operation",
                mtc_stage_type="GripperOperation",
                planner="GripperProfile",
                gripper_profile=profile,
                gripper_action=("grasp" if actor == "leader" else "move") if config.simulation_cable_config else "",
                gripper_width_override=cable_widths.get(actor),
                phase="preparation",
                confirmation_required=config.interactive and not config.simulation_cable_config,
                info=f"close {actor} gripper",
            )
        )

    if config.simulation_cable_config:
        simulation_operation("release_verify", VERIFY_USB,
            "require sustained two-finger contact, remove world support, verify grasp, then attach only in planning")

    child_specs = []
    for actor in ("leader", "follower"):
        index, group, ik_frame, _direction, _profile = actor_settings[actor]
        keypoint = keypoints[index]
        child_specs.append(
            MtcStageSpec(
                step_index=len(specs),
                stage_index=start_stage_index + len(specs),
                stage_key=(
                    f"preparation:dual:{DUAL_DESCENT}:{actor}"
                ),
                action="preparation",
                actor=actor,
                group=group,
                ik_frame=ik_frame,
                name=f"preparation_{actor}_cartesian_descent",
                from_index=index,
                to_index=index,
                from_keypoint=f"{keypoint.name}_above",
                to_keypoint=keypoint.name,
                frame_id=keypoint.frame_id,
                vector=(0.0, 0.0, -height),
                execution_order=[DUAL_DESCENT],
                primitive="cartesian_descent",
                mtc_stage_type="MoveRelative",
                planner="CartesianPath",
                phase="preparation",
                info=f"move {actor} TCP down {height:.3f} m",
            )
        )
    specs.append(
        MtcStageSpec(
            step_index=len(specs),
            stage_index=start_stage_index + len(specs),
            stage_key=f"preparation:dual:{DUAL_DESCENT}",
            action="preparation",
            actor="dual",
            group="",
            ik_frame="",
            name="preparation_dual_cartesian_descent",
            from_index=-1,
            to_index=-1,
            from_keypoint="initial_keypoints_above",
            to_keypoint="initial_keypoints",
            frame_id="",
            vector=(0.0, 0.0, -height),
            execution_order=[DUAL_DESCENT],
            primitive=DUAL_DESCENT,
            mtc_stage_type="Merger",
            planner="CartesianPath",
            phase="preparation",
            confirmation_required=config.interactive,
            children=tuple(child_specs),
            info=(
                ("Grasp verified and world support removed; start the task: "
                 if config.simulation_cable_config else "Start the task: ") +
                f"move both TCPs down {height:.3f} m on synchronized "
                "Cartesian paths"
            ),
        )
    )
    return specs


def confirm_stage(
    spec: MtcStageSpec,
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
        spec.info or spec.name,
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
            logger.warning("preparation aborted by operator before %s", spec.stage_key)
            return False
        logger.warning("unrecognized input %r; press Enter or q", answer)


def read_controlling_terminal(_prompt: str) -> str:
    """
    Read a command from the terminal that owns the ros2 launch process.

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
