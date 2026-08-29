from dual_fr3_trunking_mtc.models import Keypoint
from dual_fr3_trunking_mtc.planner import build_segment_plans, plan_to_dict
from dual_fr3_trunking_mtc.scheduler import build_task_schedule

TASK_FRAME = "left_fr3_link0"


def _keypoint(name: str, in_slot: bool) -> Keypoint:
    return Keypoint(name, TASK_FRAME, (0.0, 0.0, 0.0), in_slot)


def test_build_task_schedule_for_current_trunking_flow():
    keypoints = [
        _keypoint("kp0", False),
        _keypoint("kp1", True),
        _keypoint("kp2", True),
        _keypoint("kp3", True),
        _keypoint("kp4", True),
        _keypoint("kp5", False),
    ]

    steps = build_task_schedule(
        keypoints,
        initial_leader_index=1,
        initial_follower_index=0,
    )

    assert [step.action for step in steps] == [
        "seat_edge",
        "move_anchor",
        "straighten",
        "straighten",
        "straighten",
        "seat_edge",
    ]
    assert steps[0].leader_hold_index == 1
    assert steps[0].leader_mode == "move_ahead_for_clearance"
    assert steps[0].follower_from_index == 0
    assert steps[0].follower_to_index == 1
    assert steps[0].execution_order == [
        "follower_seat_cable_on_edge",
        "leader_move_ahead_for_clearance",
        "leader_hold_tension",
    ]
    assert steps[1].leader_from_index == 1
    assert steps[1].leader_to_index == 5
    assert steps[1].follower_hold_index == 1
    assert steps[1].execution_order == [
        "follower_hold_position",
        "leader_move_to_anchor",
        "leader_establish_tension",
    ]
    assert steps[2].leader_hold_index == 5
    assert steps[2].follower_from_index == 1
    assert steps[2].follower_to_index == 2
    assert steps[-1].leader_hold_index == 5
    assert steps[-1].follower_from_index == 4
    assert steps[-1].follower_to_index == 5
    assert steps[-1].leader_mode == "hold_position"
    assert steps[-1].execution_order == ["follower_seat_cable_on_edge"]


def test_build_task_schedule_preserves_anchor_jump_then_follower_catchup():
    keypoints = [
        _keypoint("kp0", False),
        _keypoint("kp1", True),
        _keypoint("kp2", True),
    ]

    steps = build_task_schedule(
        keypoints,
        initial_leader_index=0,
        initial_follower_index=0,
    )

    assert [step.action for step in steps] == [
        "move_anchor",
        "seat_edge",
        "move_anchor",
        "straighten",
    ]
    assert steps[0].leader_from_index == 0
    assert steps[0].leader_to_index == 1
    assert steps[1].follower_from_index == 0
    assert steps[1].follower_to_index == 1
    assert steps[2].leader_from_index == 1
    assert steps[2].leader_to_index == 2
    assert steps[3].leader_hold_index == 2
    assert steps[3].follower_from_index == 1
    assert steps[3].follower_to_index == 2


def test_leader_anchors_before_follower_straightens_when_both_are_in_slot():
    keypoints = [
        _keypoint("corner_1", True),
        _keypoint("corner_2", True),
        _keypoint("entry_5", False),
    ]

    steps = build_task_schedule(
        keypoints,
        initial_leader_index=1,
        initial_follower_index=0,
    )

    assert steps[0].action == "move_anchor"
    assert steps[0].leader_from_index == 1
    assert steps[0].leader_to_index == 2
    assert steps[1].action == "straighten"
    assert steps[1].follower_from_index == 0
    assert steps[1].follower_to_index == 1


def test_plan_summary_includes_task_schedule():
    keypoints = [
        _keypoint("kp0", False),
        _keypoint("kp1", True),
    ]
    segments = build_segment_plans(keypoints)
    steps = build_task_schedule(
        keypoints,
        initial_leader_index=1,
        initial_follower_index=0,
    )

    summary = plan_to_dict(keypoints, segments, steps)

    assert summary["task_schedule"][0]["action"] == "seat_edge"
    assert summary["task_schedule"][0]["leader"]["hold"] == "kp1"
    assert summary["task_schedule"][0]["leader"]["mode"] == "hold_position"
    assert summary["task_schedule"][0]["follower"]["from"] == "kp0"
    assert summary["task_schedule"][0]["follower"]["to"] == "kp1"
    assert summary["task_schedule"][0]["execution_order"] == [
        "follower_seat_cable_on_edge",
    ]
