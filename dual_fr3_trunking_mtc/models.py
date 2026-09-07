from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, cos, hypot, pi, sin
from typing import Any, Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped, Quaternion


DEFAULT_TOOL_ROLL = pi
DEFAULT_TOOL_PITCH = 0.0
DEFAULT_LEADER_LEAD_DISTANCE = 0.10
ORIENTATION_DIRECTION_FORWARD = "forward"
ORIENTATION_DIRECTION_REVERSE = "reverse"
ORIENTATION_DIRECTIONS = (
    ORIENTATION_DIRECTION_FORWARD,
    ORIENTATION_DIRECTION_REVERSE,
)
DEFAULT_LEADER_ORIENTATION_DIRECTION = ORIENTATION_DIRECTION_REVERSE
DEFAULT_FOLLOWER_ORIENTATION_DIRECTION = ORIENTATION_DIRECTION_FORWARD


def validate_orientation_direction(direction: str) -> str:
    if direction not in ORIENTATION_DIRECTIONS:
        expected = ", ".join(ORIENTATION_DIRECTIONS)
        raise ValueError(
            f"orientation direction must be one of: {expected}; got {direction!r}"
        )
    return direction


def path_orientation_yaw(
    delta_x: float,
    delta_y: float,
    direction: str = ORIENTATION_DIRECTION_FORWARD,
) -> float:
    """Return path yaw, optionally facing opposite the increasing point order."""
    validate_orientation_direction(direction)
    if hypot(delta_x, delta_y) < 1e-9:
        return 0.0
    if direction == ORIENTATION_DIRECTION_REVERSE:
        delta_x = -delta_x
        delta_y = -delta_y
    return atan2(delta_y, delta_x)


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)

    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


@dataclass(frozen=True)
class Keypoint:
    name: str
    frame_id: str
    position: Tuple[float, float, float]
    in_slot: bool
    role: str = "waypoint"
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SegmentPlan:
    index: int
    start: Keypoint
    goal: Keypoint
    action: str
    path_type: str
    leader_target: PoseStamped
    follower_target: PoseStamped
    execution_order: List[str] = field(default_factory=list)
    waypoints: List[PoseStamped] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)


@dataclass
class TaskStep:
    index: int
    action: str
    leader_mode: str
    follower_mode: str
    execution_order: List[str]
    segment_index: Optional[int] = None
    leader_from_index: Optional[int] = None
    leader_to_index: Optional[int] = None
    leader_hold_index: Optional[int] = None
    follower_from_index: Optional[int] = None
    follower_to_index: Optional[int] = None
    follower_hold_index: Optional[int] = None
    notes: List[str] = field(default_factory=list)


@dataclass(frozen=True)
class TaskPlan:
    """Single planning boundary passed from scheduling to compilation."""

    keypoints: Tuple[Keypoint, ...]
    segments: Tuple[SegmentPlan, ...]
    steps: Tuple[TaskStep, ...]
    initial_leader_index: int
    initial_follower_index: int
