import pytest

from dual_fr3_trunking_mtc.models import Keypoint
from dual_fr3_trunking_mtc.planner import (
    build_segment_plans,
    classify_segment,
    load_keypoints,
    plan_to_dict,
)

TASK_FRAME = "left_fr3_link0"


def test_classify_segment_same_state():
    start = Keypoint("a", TASK_FRAME, (0.0, 0.0, 0.0), False)
    goal = Keypoint("b", TASK_FRAME, (1.0, 0.0, 0.0), False)
    action, path_type = classify_segment(start, goal)
    assert action == "straighten"
    assert path_type == "cartesian"


def test_classify_segment_slot_change():
    start = Keypoint("a", TASK_FRAME, (0.0, 0.0, 0.0), False)
    goal = Keypoint("b", TASK_FRAME, (1.0, 0.0, 0.0), True)
    action, path_type = classify_segment(start, goal)
    assert action == "seat_edge"
    assert path_type == "hybrid"


def test_classify_segment_reverse_slot_change():
    start = Keypoint("a", TASK_FRAME, (0.0, 0.0, 0.0), True)
    goal = Keypoint("b", TASK_FRAME, (1.0, 0.0, 0.0), False)
    action, path_type = classify_segment(start, goal)
    assert action == "seat_edge"
    assert path_type == "hybrid"


def test_build_segment_plans():
    keypoints = [
        Keypoint("a", TASK_FRAME, (0.0, 0.0, 0.0), False),
        Keypoint("b", TASK_FRAME, (1.0, 0.0, 0.0), False),
        Keypoint("c", TASK_FRAME, (2.0, 0.0, 0.0), True),
    ]
    segments = build_segment_plans(keypoints, samples_per_segment=5)
    assert len(segments) == 2
    assert segments[0].action == "straighten"
    assert segments[0].execution_order == [
        "leader_move_to_segment_goal",
        "follower_follow_segment_path",
    ]
    assert segments[1].action == "seat_edge"
    assert segments[1].execution_order == [
        "leader_move_ahead_for_clearance",
        "follower_seat_cable_on_edge",
        "leader_hold_tension",
    ]
    assert len(segments[0].waypoints) == 5


def test_plan_summary_includes_execution_order():
    keypoints = [
        Keypoint("a", TASK_FRAME, (0.0, 0.0, 0.0), False),
        Keypoint("b", TASK_FRAME, (1.0, 0.0, 0.0), True),
    ]

    segments = build_segment_plans(keypoints, samples_per_segment=5)
    summary = plan_to_dict(keypoints, segments)

    assert "rpy" not in summary["keypoints"][0]
    assert summary["segments"][0]["action"] == "seat_edge"
    assert summary["segments"][0]["execution_order"] == [
        "leader_move_ahead_for_clearance",
        "follower_seat_cable_on_edge",
        "leader_hold_tension",
    ]


def test_build_segment_rejects_mixed_frames():
    keypoints = [
        Keypoint("a", TASK_FRAME, (0.0, 0.0, 0.0), False),
        Keypoint("b", "world", (1.0, 0.0, 0.0), False),
    ]

    try:
        build_segment_plans(keypoints, samples_per_segment=5)
    except ValueError as exc:
        assert "different frames" in str(exc)
    else:
        raise AssertionError("mixed-frame segment should fail")


def test_load_keypoints_rejects_point_orientation(tmp_path):
    config = tmp_path / "keypoints.yaml"
    config.write_text(
        """
keypoints:
  - name: point_with_direction
    position: [0.1, 0.2, 0.3]
    rpy: [3.14159, 0.0, 0.0]
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="must not define rpy"):
        load_keypoints(config)
