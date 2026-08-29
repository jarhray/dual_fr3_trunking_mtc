import math

import pytest

from dual_fr3_trunking_mtc.models import Keypoint
from dual_fr3_trunking_mtc.mtc_prototype import (
    build_mtc_stage_specs,
    mtc_stage_sequence_to_dict,
    mtc_stage_sequence_to_text,
)
from dual_fr3_trunking_mtc.scheduler import build_task_schedule
from dual_fr3_trunking_mtc.segment_executor import (
    build_segment_stages,
    stage_description,
)

TASK_FRAME = "left_fr3_link0"


def _keypoint(name: str, x: float, y: float, in_slot: bool) -> Keypoint:
    return Keypoint(name, TASK_FRAME, (x, y, 0.0), in_slot)


def test_build_mtc_stage_specs_from_task_schedule():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.0, 0.1, True),
        _keypoint("kp2", 0.0, 0.2, True),
        _keypoint("kp3", 0.0, 0.3, False),
    ]
    task_steps = build_task_schedule(
        keypoints,
        initial_leader_index=1,
        initial_follower_index=0,
    )

    specs = build_mtc_stage_specs(keypoints, task_steps)

    assert [spec.primitive for spec in specs] == [
        "close_gripper_at_start",
        "close_gripper_at_start",
        "move_to_initial_keypoint",
        "move_to_initial_keypoint",
        "seat_cable_on_edge",
        "turn_gripper_to_next_keypoint",
        "leader_move_ahead_for_seat_edge",
        "turn_gripper_to_next_keypoint",
        "direct_move_to_seat_edge_keypoint",
        "direct_move_to_next_anchor",
        "turn_gripper_to_next_keypoint",
        "cartesian_move_to_next_keypoint",
        "seat_cable_on_edge",
    ]
    assert specs[0].group == "right_fr3_hand"
    assert specs[0].joint_goal == {"right_fr3_finger_joint1": 0.0}
    assert specs[1].group == "left_fr3_hand"
    assert specs[2].group == "right_fr3_arm"
    assert specs[3].group == "left_fr3_arm"
    assert specs[4].executable is False
    assert specs[6].vector == pytest.approx((0.0, 0.10, 0.0))
    assert specs[9].primitive == "direct_move_to_next_anchor"
    assert specs[10].planner == "CartesianPath"
    assert specs[12].executable is False


def test_mtc_stage_sequence_interface_includes_execution_order():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.0, 0.1, True),
    ]
    task_steps = build_task_schedule(
        keypoints,
        initial_leader_index=1,
        initial_follower_index=0,
    )
    specs = build_mtc_stage_specs(keypoints, task_steps)

    sequence = mtc_stage_sequence_to_dict(specs)
    text = mtc_stage_sequence_to_text(specs)

    assert sequence["interface_version"] == 1
    assert sequence["stage_count"] == 5
    assert sequence["stage_sequence"][0]["mtc_stage_type"] == "MoveTo"
    assert sequence["stage_sequence"][0]["primitive"] == "close_gripper_at_start"
    assert sequence["stage_sequence"][0]["group"] == "right_fr3_hand"
    assert sequence["stage_sequence"][0]["actor"] == "follower"
    assert sequence["stage_sequence"][1]["primitive"] == "close_gripper_at_start"
    assert sequence["stage_sequence"][1]["group"] == "left_fr3_hand"
    assert sequence["stage_sequence"][2]["primitive"] == "move_to_initial_keypoint"
    assert sequence["stage_sequence"][2]["actor"] == "follower"
    assert sequence["stage_sequence"][2]["to"] == "kp0"
    assert sequence["stage_sequence"][3]["actor"] == "leader"
    assert sequence["stage_sequence"][3]["to"] == "kp1"
    assert sequence["stage_sequence"][0]["execution_order"] == [
        "follower_close_gripper",
    ]
    assert sequence["stage_sequence"][4]["primitive"] == "seat_cable_on_edge"
    assert sequence["stage_sequence"][4]["execution_order"] == [
        "follower_seat_cable_on_edge",
    ]
    assert "stage_key=step_0:seat_edge:follower:kp0->kp1:seat_cable_on_edge" in text


def test_leader_lead_distance_must_be_positive():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.1, 0.0, True),
    ]
    task_steps = build_task_schedule(keypoints)

    with pytest.raises(ValueError, match="leader_lead_distance"):
        build_mtc_stage_specs(
            keypoints,
            task_steps,
            leader_lead_distance=0.0,
        )


def test_grippers_close_before_optional_initial_alignment():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.0, 0.1, True),
    ]
    specs = build_mtc_stage_specs(
        keypoints,
        build_task_schedule(keypoints),
        align_initial_poses=False,
    )

    assert [spec.primitive for spec in specs[:2]] == [
        "close_gripper_at_start",
        "close_gripper_at_start",
    ]
    assert all(spec.primitive != "move_to_initial_keypoint" for spec in specs)


def test_turn_specs_use_actor_current_yaw():
    keypoints = [
        _keypoint("entry_0", 0.6, 0.70, False),
        _keypoint("corner_1", 0.6, 0.45, True),
        _keypoint("corner_2", 0.6, 0.35, True),
        _keypoint("corner_3", 0.4, 0.35, True),
        _keypoint("corner_4", 0.4, 0.25, True),
        _keypoint("entry_5", 0.4, 0.15, False),
    ]
    specs = build_mtc_stage_specs(keypoints, build_task_schedule(keypoints))
    turns = [
        spec for spec in specs
        if spec.primitive == "turn_gripper_to_next_keypoint"
    ]

    assert [turn.yaw_delta for turn in turns] == pytest.approx(
        [
            0.0,
            0.0,
            0.0,
            -math.pi / 2.0,
            math.pi / 2.0,
        ]
    )
    assert [turn.executable for turn in turns] == [
        False,
        False,
        False,
        True,
        True,
    ]


def test_segment_executor_uses_the_mtc_stage_order():
    keypoints = [
        _keypoint("entry_0", 0.6, 0.70, False),
        _keypoint("corner_1", 0.6, 0.45, True),
        _keypoint("corner_2", 0.6, 0.35, True),
        _keypoint("corner_3", 0.4, 0.35, True),
        _keypoint("corner_4", 0.4, 0.25, True),
        _keypoint("entry_5", 0.4, 0.15, False),
    ]

    specs = build_segment_stages(keypoints)

    assert len(specs) == 17
    assert specs[0].primitive == "close_gripper_at_start"
    assert specs[1].primitive == "close_gripper_at_start"
    assert specs[2].actor == "follower"
    assert specs[2].to_keypoint == "entry_0"
    assert specs[6].primitive == "leader_move_ahead_for_seat_edge"
    assert specs[9].primitive == "direct_move_to_next_anchor"
    assert specs[-1].primitive == "seat_cable_on_edge"
    assert "cartesian" in stage_description(specs[13])
