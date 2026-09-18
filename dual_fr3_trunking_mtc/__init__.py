"""Dual FR3 trunking planning helpers."""

# Keep legacy modules importable without loading ROS, CUDA or PhysX.
from pathlib import Path as _Path

__path__.append(str(_Path(__file__).parent / "_compat"))

from dual_fr3_trunking_mtc.task.models import (
    Keypoint,
    SegmentPlan,
    TaskPlan,
    TaskStep,
    rpy_to_quaternion,
)
from dual_fr3_trunking_mtc.task.planner import build_segment_plans, classify_segment, load_keypoints
from dual_fr3_trunking_mtc.task.scheduler import build_task_plan, build_task_schedule

__all__ = [
    "Keypoint",
    "SegmentPlan",
    "TaskPlan",
    "TaskStep",
    "build_segment_plans",
    "build_task_plan",
    "build_task_schedule",
    "classify_segment",
    "load_keypoints",
    "rpy_to_quaternion",
]
