from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Sequence

import yaml
from geometry_msgs.msg import PoseStamped

from .models import (
    DEFAULT_FOLLOWER_ORIENTATION_DIRECTION,
    DEFAULT_LEADER_ORIENTATION_DIRECTION,
    DEFAULT_TOOL_PITCH,
    DEFAULT_TOOL_ROLL,
    Keypoint,
    ORIENTATION_DIRECTION_FORWARD,
    SegmentPlan,
    TaskPlan,
    TaskStep,
    path_orientation_yaw,
    rpy_to_quaternion,
    validate_orientation_direction,
)


def _as_float_triplet(
    values: Sequence[Any], field_name: str
) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{field_name} must have exactly 3 values")
    return (float(values[0]), float(values[1]), float(values[2]))


def _interp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _load_yaml_items(
    raw: Any,
    fallback_frame: str = "left_fr3_link0",
) -> tuple[str, List[Dict[str, Any]]]:
    default_frame = fallback_frame
    if isinstance(raw, dict):
        default_frame = str(raw.get("default_frame", default_frame))
        keypoints = raw.get("keypoints", [])
    elif isinstance(raw, list):
        keypoints = raw
    else:
        keypoints = []
    if not isinstance(keypoints, list):
        raise ValueError("keypoints must be a list")
    return default_frame, keypoints


def load_keypoints(
    path: str | Path,
    fallback_frame: str = "left_fr3_link0",
) -> List[Keypoint]:
    with Path(path).expanduser().open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    default_frame, items = _load_yaml_items(raw, fallback_frame=fallback_frame)
    keypoints: List[Keypoint] = []

    for idx, item in enumerate(items):
        if not isinstance(item, dict):
            raise ValueError(f"keypoint #{idx} must be a mapping")

        name = str(item.get("name", f"p{idx:02d}"))
        frame_id = str(item.get("frame_id", default_frame))
        position = _as_float_triplet(item.get("position", [0.0, 0.0, 0.0]), "position")
        if "rpy" in item:
            raise ValueError(
                f"keypoint '{name}' must not define rpy; tool orientation is "
                "derived from the path"
            )
        in_slot = bool(item.get("in_slot", False))
        role = str(item.get("role", "waypoint"))
        metadata = dict(item.get("metadata", {}))

        keypoints.append(
            Keypoint(
                name=name,
                frame_id=frame_id,
                position=position,
                in_slot=in_slot,
                role=role,
                metadata=metadata,
            )
        )

    return keypoints


def classify_segment(start: Keypoint, goal: Keypoint) -> tuple[str, str]:
    if start.in_slot == goal.in_slot:
        return "straighten", "cartesian"
    return "seat_edge", "hybrid"


def execution_order_for_action(action: str) -> List[str]:
    if action == "seat_edge":
        return [
            "leader_move_ahead_for_clearance",
            "follower_seat_cable_on_edge",
            "leader_hold_tension",
        ]

    return [
        "leader_move_to_segment_goal",
        "follower_follow_segment_path",
    ]


def _segment_yaw(
    start: Keypoint,
    goal: Keypoint,
    orientation_direction: str = ORIENTATION_DIRECTION_FORWARD,
) -> float:
    delta_x = goal.position[0] - start.position[0]
    delta_y = goal.position[1] - start.position[1]
    return path_orientation_yaw(delta_x, delta_y, orientation_direction)


def _pose_stamped_at(
    keypoint: Keypoint,
    yaw: float,
    tool_roll: float = DEFAULT_TOOL_ROLL,
    tool_pitch: float = DEFAULT_TOOL_PITCH,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = keypoint.frame_id
    pose.pose.position.x = keypoint.position[0]
    pose.pose.position.y = keypoint.position[1]
    pose.pose.position.z = keypoint.position[2]
    pose.pose.orientation = rpy_to_quaternion(tool_roll, tool_pitch, yaw)
    return pose


def _interpolate_pose_stamped(start: Keypoint, goal: Keypoint, t: float) -> PoseStamped:
    if start.frame_id != goal.frame_id:
        raise ValueError(
            f"Cannot interpolate between different frames: "
            f"{start.name} is in {start.frame_id}, "
            f"{goal.name} is in {goal.frame_id}"
        )

    pose = PoseStamped()
    pose.header.frame_id = goal.frame_id
    pose.pose.position.x = _interp(start.position[0], goal.position[0], t)
    pose.pose.position.y = _interp(start.position[1], goal.position[1], t)
    pose.pose.position.z = _interp(start.position[2], goal.position[2], t)
    pose.pose.orientation = rpy_to_quaternion(
        DEFAULT_TOOL_ROLL,
        DEFAULT_TOOL_PITCH,
        _segment_yaw(start, goal),
    )
    return pose


def sample_segment(start: Keypoint, goal: Keypoint, samples: int = 8) -> List[PoseStamped]:
    if samples < 2:
        samples = 2
    return [_interpolate_pose_stamped(start, goal, i / (samples - 1)) for i in range(samples)]


def build_segment_plans(
    keypoints: Sequence[Keypoint],
    samples_per_segment: int = 8,
    leader_orientation_direction: str = DEFAULT_LEADER_ORIENTATION_DIRECTION,
    follower_orientation_direction: str = DEFAULT_FOLLOWER_ORIENTATION_DIRECTION,
) -> List[SegmentPlan]:
    validate_orientation_direction(leader_orientation_direction)
    validate_orientation_direction(follower_orientation_direction)
    plans: List[SegmentPlan] = []
    if len(keypoints) < 2:
        return plans

    for index, (start, goal) in enumerate(zip(keypoints[:-1], keypoints[1:])):
        action, path_type = classify_segment(start, goal)
        waypoints = sample_segment(start, goal, samples=samples_per_segment)
        notes = [
            f"{start.name} -> {goal.name}",
            f"in_slot: {start.in_slot} -> {goal.in_slot}",
        ]
        leader_target = _pose_stamped_at(
            goal,
            _segment_yaw(start, goal, leader_orientation_direction),
        )
        follower_target = _pose_stamped_at(
            goal,
            _segment_yaw(start, goal, follower_orientation_direction),
        )
        execution_order = execution_order_for_action(action)
        notes.append(f"execution_order: {' -> '.join(execution_order)}")
        plans.append(
            SegmentPlan(
                index=index,
                start=start,
                goal=goal,
                action=action,
                path_type=path_type,
                leader_target=leader_target,
                follower_target=follower_target,
                execution_order=execution_order,
                waypoints=waypoints,
                notes=notes,
            )
        )

    return plans


def _task_step_to_dict(
    step: TaskStep,
    keypoints: Sequence[Keypoint],
) -> Dict[str, Any]:
    def name_or_none(index: int | None) -> str | None:
        if index is None:
            return None
        return keypoints[index].name

    return {
        "index": step.index,
        "action": step.action,
        "segment_index": step.segment_index,
        "leader": {
            "mode": step.leader_mode,
            "from_index": step.leader_from_index,
            "from": name_or_none(step.leader_from_index),
            "to_index": step.leader_to_index,
            "to": name_or_none(step.leader_to_index),
            "hold_index": step.leader_hold_index,
            "hold": name_or_none(step.leader_hold_index),
        },
        "follower": {
            "mode": step.follower_mode,
            "from_index": step.follower_from_index,
            "from": name_or_none(step.follower_from_index),
            "to_index": step.follower_to_index,
            "to": name_or_none(step.follower_to_index),
            "hold_index": step.follower_hold_index,
            "hold": name_or_none(step.follower_hold_index),
        },
        "execution_order": list(step.execution_order),
        "notes": list(step.notes),
    }


def plan_to_dict(task_plan: TaskPlan) -> Dict[str, Any]:
    keypoints = task_plan.keypoints
    segments = task_plan.segments
    summary = {
        "keypoints": [
            {
                "name": item.name,
                "frame_id": item.frame_id,
                "position": list(item.position),
                "in_slot": item.in_slot,
                "role": item.role,
            }
            for item in keypoints
        ],
        "segments": [
            {
                "index": segment.index,
                "start": segment.start.name,
                "goal": segment.goal.name,
                "action": segment.action,
                "path_type": segment.path_type,
                "start_in_slot": segment.start.in_slot,
                "goal_in_slot": segment.goal.in_slot,
                "execution_order": list(segment.execution_order),
                "waypoints": len(segment.waypoints),
                "notes": list(segment.notes),
            }
            for segment in segments
        ],
    }

    summary["task_schedule"] = [
        _task_step_to_dict(step, keypoints)
        for step in task_plan.steps
    ]

    return summary


def plan_to_json(task_plan: TaskPlan) -> str:
    return json.dumps(
        plan_to_dict(task_plan),
        ensure_ascii=False,
        indent=2,
    )
