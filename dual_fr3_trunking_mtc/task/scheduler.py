from __future__ import annotations

from typing import List, Optional, Sequence

from dual_fr3_trunking_mtc.task.models import Keypoint, SegmentPlan, TaskPlan, TaskStep


SUPPORTED_SEGMENT_ACTIONS = {"straighten", "seat_edge"}


def _keypoints_from_segments(
    segments: Sequence[SegmentPlan],
) -> tuple[Keypoint, ...]:
    """Validate an ordered SegmentPlan chain and recover its keypoints."""
    if not segments:
        return ()

    keypoints = [segments[0].start]
    for expected_index, segment in enumerate(segments):
        if segment.index != expected_index:
            raise ValueError(
                "segments must be ordered with contiguous zero-based indices; "
                f"expected {expected_index}, got {segment.index}"
            )
        if segment.start != keypoints[-1]:
            raise ValueError(
                "segments must form a continuous chain; "
                f"segment {segment.index} starts at {segment.start.name!r}, "
                f"expected {keypoints[-1].name!r}"
            )
        if segment.action not in SUPPORTED_SEGMENT_ACTIONS:
            raise ValueError(
                f"unsupported segment action {segment.action!r} at "
                f"segment {segment.index}"
            )
        keypoints.append(segment.goal)
    return tuple(keypoints)


def _validate_indices(
    keypoint_count: int,
    initial_leader_index: int,
    initial_follower_index: int,
) -> None:
    if keypoint_count == 0:
        return

    last_index = keypoint_count - 1
    if not 0 <= initial_leader_index <= last_index:
        raise ValueError(
            f"initial_leader_index must be between 0 and {last_index}, "
            f"got {initial_leader_index}"
        )
    if not 0 <= initial_follower_index <= last_index:
        raise ValueError(
            f"initial_follower_index must be between 0 and {last_index}, "
            f"got {initial_follower_index}"
        )
    if initial_follower_index > initial_leader_index:
        raise ValueError(
            "initial_follower_index must not be ahead of initial_leader_index"
        )


def _follower_step(
    step_index: int,
    segments: Sequence[SegmentPlan],
    keypoints: Sequence[Keypoint],
    follower_index: int,
    leader_anchor_index: int,
) -> TaskStep:
    segment = segments[follower_index]
    start = segment.start
    goal = segment.goal
    action = segment.action

    if action == "seat_edge":
        terminal_seat_edge = (
            follower_index + 1 == len(keypoints) - 1
            and leader_anchor_index == len(keypoints) - 1
        )
        execution_order = ["follower_seat_cable_on_edge"]
        if not terminal_seat_edge:
            execution_order.extend(
                [
                    "leader_move_ahead_for_clearance",
                    "leader_hold_tension",
                ]
            )
        follower_mode = "seat_edge"
        leader_mode = (
            "hold_position"
            if terminal_seat_edge
            else "move_ahead_for_clearance"
        )
    else:
        execution_order = [
            "leader_hold_anchor",
            "follower_follow_segment_path",
        ]
        follower_mode = "follow_segment"
        leader_mode = "hold_anchor"

    if action == "seat_edge" and leader_mode == "hold_position":
        leader_note = (
            f"leader already holds final point "
            f"{keypoints[leader_anchor_index].name}"
        )
    elif action == "seat_edge":
        leader_note = f"leader clears {keypoints[leader_anchor_index].name}"
    else:
        leader_note = f"leader holds {keypoints[leader_anchor_index].name}"

    return TaskStep(
        index=step_index,
        action=action,
        segment_index=follower_index,
        leader_mode=leader_mode,
        leader_hold_index=leader_anchor_index,
        follower_mode=follower_mode,
        follower_from_index=follower_index,
        follower_to_index=follower_index + 1,
        execution_order=execution_order,
        notes=[
            leader_note,
            f"follower moves {start.name} -> {goal.name}",
        ],
    )


def _next_anchor_index(
    segments: Sequence[SegmentPlan],
    current_leader_index: int,
) -> Optional[int]:
    for segment in segments[current_leader_index:]:
        if segment.action == "seat_edge":
            return segment.index + 1

    last_index = len(segments)
    if current_leader_index < last_index:
        return last_index
    return None


def _leader_anchor_step(
    step_index: int,
    keypoints: Sequence[Keypoint],
    leader_index: int,
    anchor_index: int,
    follower_index: int,
) -> TaskStep:
    return TaskStep(
        index=step_index,
        action="move_anchor",
        leader_mode="move_to_anchor",
        leader_from_index=leader_index,
        leader_to_index=anchor_index,
        follower_mode="hold_position",
        follower_hold_index=follower_index,
        execution_order=[
            "follower_hold_position",
            "leader_move_to_anchor",
            "leader_establish_tension",
        ],
        notes=[
            f"leader moves {keypoints[leader_index].name} -> "
            f"{keypoints[anchor_index].name}",
            f"follower holds {keypoints[follower_index].name}",
        ],
    )


def build_task_plan(
    segments: Sequence[SegmentPlan],
    initial_leader_index: int = 1,
    initial_follower_index: int = 0,
) -> TaskPlan:
    """Schedule an already classified segment chain into a complete task plan."""
    segment_chain = tuple(segments)
    keypoints = _keypoints_from_segments(segment_chain)
    if not segment_chain:
        return TaskPlan(
            keypoints=(),
            segments=(),
            steps=(),
            initial_leader_index=initial_leader_index,
            initial_follower_index=initial_follower_index,
        )
    _validate_indices(
        len(keypoints),
        initial_leader_index,
        initial_follower_index,
    )

    leader_index = initial_leader_index
    follower_index = initial_follower_index
    steps: List[TaskStep] = []

    while True:
        while follower_index < leader_index:
            follower_action = segment_chain[follower_index].action
            if follower_action != "seat_edge":
                next_anchor = _next_anchor_index(segment_chain, leader_index)
                if next_anchor is not None:
                    break
            steps.append(
                _follower_step(
                    len(steps),
                    segment_chain,
                    keypoints,
                    follower_index,
                    leader_index,
                )
            )
            follower_index += 1

        next_anchor = _next_anchor_index(segment_chain, leader_index)
        if next_anchor is None:
            break

        steps.append(
            _leader_anchor_step(
                len(steps),
                keypoints,
                leader_index,
                next_anchor,
                follower_index,
            )
        )
        leader_index = next_anchor

    return TaskPlan(
        keypoints=keypoints,
        segments=segment_chain,
        steps=tuple(steps),
        initial_leader_index=initial_leader_index,
        initial_follower_index=initial_follower_index,
    )


def build_task_schedule(
    segments: Sequence[SegmentPlan],
    initial_leader_index: int = 1,
    initial_follower_index: int = 0,
) -> List[TaskStep]:
    """Compatibility view of :func:`build_task_plan` returning only its steps."""
    return list(
        build_task_plan(
            segments,
            initial_leader_index=initial_leader_index,
            initial_follower_index=initial_follower_index,
        ).steps
    )
