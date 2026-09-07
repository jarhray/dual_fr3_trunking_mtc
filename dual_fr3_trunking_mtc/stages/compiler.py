from __future__ import annotations

import math
from dataclasses import replace
from typing import Any, Dict, Sequence

from ..models import (
    DEFAULT_FOLLOWER_ORIENTATION_DIRECTION,
    DEFAULT_LEADER_LEAD_DISTANCE,
    DEFAULT_LEADER_ORIENTATION_DIRECTION,
    Keypoint,
    ORIENTATION_DIRECTION_FORWARD,
    TaskPlan,
    TaskStep,
    path_orientation_yaw,
    validate_orientation_direction,
)
from ..preparation import PreparationConfig, build_preparation_stage_specs
from .specs import MtcStageSpec


def _index_pair_for_step(step: TaskStep) -> tuple[str, int | None, int | None]:
    if step.leader_from_index is not None and step.leader_to_index is not None:
        return "leader", step.leader_from_index, step.leader_to_index
    if step.follower_from_index is not None and step.follower_to_index is not None:
        return "follower", step.follower_from_index, step.follower_to_index
    return "none", None, None


def _delta(start: Keypoint, goal: Keypoint) -> tuple[float, float, float]:
    if start.frame_id != goal.frame_id:
        raise ValueError(
            "Cannot create MTC relative move between different frames: "
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


def _yaw_toward(
    vector: tuple[float, float, float],
    orientation_direction: str = ORIENTATION_DIRECTION_FORWARD,
) -> float:
    return path_orientation_yaw(
        vector[0],
        vector[1],
        orientation_direction,
    )


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


def _path_yaw(
    keypoints: Sequence[Keypoint],
    index: int,
    orientation_direction: str = ORIENTATION_DIRECTION_FORWARD,
) -> float:
    direction = _path_direction(keypoints, index)
    return _yaw_toward(direction, orientation_direction)


def _scaled_direction(
    vector: tuple[float, float, float],
    distance: float,
) -> tuple[float, float, float]:
    length = _norm(vector)
    if length < 1e-9:
        raise ValueError("Cannot scale a zero-length path direction")
    scale = distance / length
    return tuple(component * scale for component in vector)


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


def _leader_seat_edge_turn_spec(
    stage_index: int,
    step: TaskStep,
    keypoints: Sequence[Keypoint],
    leader_group: str,
    leader_ik_frame: str,
    current_yaw: float,
    leader_orientation_direction: str,
) -> MtcStageSpec:
    if step.leader_hold_index is None:
        raise ValueError("seat_edge step requires leader_hold_index")
    hold_index = step.leader_hold_index
    hold = keypoints[hold_index]
    target_yaw = _path_yaw(
        keypoints,
        hold_index,
        leader_orientation_direction,
    )
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


def _hand_group_for_actor(
    actor: str,
    leader_group: str,
    follower_group: str,
) -> str:
    arm_group = leader_group if actor == "leader" else follower_group
    if arm_group.endswith("_arm"):
        return f"{arm_group[:-4]}_hand"
    return f"{arm_group}_hand"


def finger_joint_for_ik_frame(ik_frame: str) -> str:
    if ik_frame.endswith("hand_tcp"):
        return f"{ik_frame[:-8]}finger_joint1"
    return f"{ik_frame}_finger_joint1"


def _seat_edge_gripper_spec(
    stage_index: int,
    step: TaskStep,
    keypoints: Sequence[Keypoint],
    leader_group: str,
    follower_group: str,
    leader_ik_frame: str,
    follower_ik_frame: str,
    from_index: int,
    to_index: int,
) -> MtcStageSpec | None:
    """Build an optional profile-based gripper operation from keypoint metadata."""
    start = keypoints[from_index]
    goal = keypoints[to_index]
    raw = goal.metadata.get("gripper")
    if raw is None:
        return None
    if isinstance(raw, str):
        settings = {"profile": raw}
    elif isinstance(raw, dict):
        settings = raw
    else:
        raise ValueError(
            f"keypoint {goal.name!r} metadata.gripper must be a string or mapping"
        )
    if not bool(settings.get("enabled", True)):
        return None

    actor = str(settings.get("actor", "follower"))
    if actor not in {"leader", "follower"}:
        raise ValueError(
            f"keypoint {goal.name!r} gripper actor must be leader or follower"
        )
    profile = str(settings.get("profile", "")).strip()
    if not profile:
        raise ValueError(
            f"keypoint {goal.name!r} gripper operation requires a profile"
        )
    action_override = str(settings.get("action", "")).strip()
    width_override = settings.get("width")
    if width_override is not None:
        width_override = float(width_override)
    ik_frame = leader_ik_frame if actor == "leader" else follower_ik_frame
    return MtcStageSpec(
        step_index=step.index,
        stage_index=stage_index,
        stage_key=_stage_key(
            step,
            actor,
            start,
            goal,
            "gripper_operation",
        ),
        action=step.action,
        actor=actor,
        group=_hand_group_for_actor(actor, leader_group, follower_group),
        ik_frame=ik_frame,
        name=f"step_{step.index}_seat_edge_{actor}_gripper_{profile}",
        from_index=from_index,
        to_index=to_index,
        from_keypoint=start.name,
        to_keypoint=goal.name,
        frame_id=start.frame_id,
        vector=(0.0, 0.0, 0.0),
        execution_order=list(step.execution_order),
        primitive="gripper_operation",
        mtc_stage_type="GripperOperation",
        planner="GripperProfile",
        gripper_profile=profile,
        gripper_action=action_override,
        gripper_width_override=width_override,
        info=f"execute reusable gripper profile {profile!r} during seat_edge",
    )


def _leader_seat_edge_lead_spec(
    stage_index: int,
    step: TaskStep,
    keypoints: Sequence[Keypoint],
    leader_group: str,
    leader_ik_frame: str,
    leader_lead_distance: float,
    target_yaw: float,
) -> MtcStageSpec:
    if step.leader_hold_index is None:
        raise ValueError("seat_edge step requires leader_hold_index")
    hold_index = step.leader_hold_index
    hold = keypoints[hold_index]
    # This is a translation along the route. The orientation may be reversed,
    # but the leader must still move ahead toward the next keypoint.
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
        target_yaw=target_yaw,
        info=(
            "leader moves ahead along the cable path to clear the seat-edge "
            "keypoint before follower motion"
        ),
    )


def _build_formal_mtc_stage_specs(
    task_plan: TaskPlan,
    leader_group: str = "left_fr3_arm",
    follower_group: str = "right_fr3_arm",
    leader_ik_frame: str = "left_fr3_hand_tcp",
    follower_ik_frame: str = "right_fr3_hand_tcp",
    leader_lead_distance: float = DEFAULT_LEADER_LEAD_DISTANCE,
    min_motion_distance: float = 1e-4,
    leader_orientation_direction: str = DEFAULT_LEADER_ORIENTATION_DIRECTION,
    follower_orientation_direction: str = DEFAULT_FOLLOWER_ORIENTATION_DIRECTION,
) -> list[MtcStageSpec]:
    """Compile the scheduled task plan into runtime-neutral MTC stage specs."""
    if leader_lead_distance <= 0.0:
        raise ValueError("leader_lead_distance must be greater than zero")
    keypoints = task_plan.keypoints
    task_steps = task_plan.steps
    orientation_directions = {
        "leader": validate_orientation_direction(leader_orientation_direction),
        "follower": validate_orientation_direction(follower_orientation_direction),
    }
    specs: list[MtcStageSpec] = []
    actor_yaw = {
        "leader": (
            _path_yaw(
                keypoints,
                task_plan.initial_leader_index,
                orientation_directions["leader"],
            )
            if 0 <= task_plan.initial_leader_index < len(keypoints)
            else 0.0
        ),
        "follower": (
            _path_yaw(
                keypoints,
                task_plan.initial_follower_index,
                orientation_directions["follower"],
            )
            if 0 <= task_plan.initial_follower_index < len(keypoints)
            else 0.0
        ),
    }
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
            gripper_spec = _seat_edge_gripper_spec(
                len(specs),
                step,
                keypoints,
                leader_group,
                follower_group,
                leader_ik_frame,
                follower_ik_frame,
                from_index,
                to_index,
            )
            if gripper_spec is not None:
                specs.append(gripper_spec)
            terminal_seat_edge = (
                to_index == len(keypoints) - 1
                and step.leader_hold_index == len(keypoints) - 1
                and from_index == to_index - 1
            )
            if terminal_seat_edge:
                continue
            leader_target_yaw = _path_yaw(
                keypoints,
                step.leader_hold_index,
                orientation_directions["leader"],
            )
            specs.append(
                _leader_seat_edge_turn_spec(
                    len(specs),
                    step,
                    keypoints,
                    leader_group,
                    leader_ik_frame,
                    actor_yaw["leader"],
                    orientation_directions["leader"],
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
                    leader_target_yaw,
                )
            )

        if step.action in {"straighten", "seat_edge"}:
            target_yaw = _yaw_toward(
                vector,
                orientation_directions[actor],
            )
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
                        "gripper is already aligned with its configured path "
                        "orientation"
                        if not turn_executable
                        else "rotate gripper in place with zero TCP "
                        "translation to its configured path orientation before "
                        "moving"
                    ),
                )
            )
            actor_yaw[actor] = target_yaw
            if step.action == "seat_edge":
                goal_yaw = _path_yaw(
                    keypoints,
                    to_index,
                    orientation_directions[actor],
                )
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
                            "keypoint using its configured path orientation"
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
                    target_yaw=target_yaw,
                    info="cartesian move to next keypoint with gripper closed",
                )
            )
            continue

        if step.action == "move_anchor":
            goal_yaw = _path_yaw(
                keypoints,
                to_index,
                orientation_directions[actor],
            )
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
                    planner="PipelinePlanner",
                    target_yaw=goal_yaw,
                    info=(
                        "leader plans through OMPL to the next anchor pose with "
                        "gripper closed using its configured path orientation"
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


def build_mtc_stage_specs(
    task_plan: TaskPlan,
    leader_group: str = "left_fr3_arm",
    follower_group: str = "right_fr3_arm",
    leader_ik_frame: str = "left_fr3_hand_tcp",
    follower_ik_frame: str = "right_fr3_hand_tcp",
    leader_lead_distance: float = DEFAULT_LEADER_LEAD_DISTANCE,
    min_motion_distance: float = 1e-4,
    leader_orientation_direction: str = DEFAULT_LEADER_ORIENTATION_DIRECTION,
    follower_orientation_direction: str = DEFAULT_FOLLOWER_ORIENTATION_DIRECTION,
    preparation_config: PreparationConfig | None = None,
) -> list[MtcStageSpec]:
    """Compile preparation and formal actions into one ordered stage program."""
    preparation_specs = (
        build_preparation_stage_specs(task_plan, preparation_config)
        if preparation_config is not None
        else []
    )
    formal_specs = _build_formal_mtc_stage_specs(
        task_plan,
        leader_group=leader_group,
        follower_group=follower_group,
        leader_ik_frame=leader_ik_frame,
        follower_ik_frame=follower_ik_frame,
        leader_lead_distance=leader_lead_distance,
        min_motion_distance=min_motion_distance,
        leader_orientation_direction=leader_orientation_direction,
        follower_orientation_direction=follower_orientation_direction,
    )
    offset = len(preparation_specs)
    return preparation_specs + [
        replace(spec, stage_index=offset + index)
        for index, spec in enumerate(formal_specs)
    ]
