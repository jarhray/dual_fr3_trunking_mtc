from __future__ import annotations

from typing import List, Optional, Sequence

from .models import Keypoint, TaskStep
from .planner import classify_segment


def _validate_indices(
    keypoints: Sequence[Keypoint],
    initial_leader_index: int,
    initial_follower_index: int,
) -> None:
    if not keypoints:
        return

    last_index = len(keypoints) - 1
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
    keypoints: Sequence[Keypoint],
    follower_index: int,
    leader_anchor_index: int,
) -> TaskStep:
    start = keypoints[follower_index]
    goal = keypoints[follower_index + 1]
    action, _path_type = classify_segment(start, goal)

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
    keypoints: Sequence[Keypoint],
    current_leader_index: int,
) -> Optional[int]:
    for index in range(current_leader_index + 1, len(keypoints)):
        if keypoints[index].in_slot != keypoints[index - 1].in_slot:
            return index

    last_index = len(keypoints) - 1
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


def build_task_schedule(
    keypoints: Sequence[Keypoint],
    initial_leader_index: int = 1,
    initial_follower_index: int = 0,
) -> List[TaskStep]:
    if len(keypoints) < 2:
        return []
    _validate_indices(keypoints, initial_leader_index, initial_follower_index)

    leader_index = initial_leader_index
    follower_index = initial_follower_index
    steps: List[TaskStep] = []

    while True:
        while follower_index < leader_index:
            follower_action, _path_type = classify_segment(
                keypoints[follower_index],
                keypoints[follower_index + 1],
            )
            if follower_action != "seat_edge":
                next_anchor = _next_anchor_index(keypoints, leader_index)
                if next_anchor is not None:
                    break
            steps.append(
                _follower_step(
                    len(steps),
                    keypoints,
                    follower_index,
                    leader_index,
                )
            )
            follower_index += 1

        next_anchor = _next_anchor_index(keypoints, leader_index)
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

    return steps
