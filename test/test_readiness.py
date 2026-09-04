from dual_fr3_trunking_mtc.readiness import (
    expected_arm_joint_names,
    gripper_command_action_name,
    missing_arm_joint_names,
)
from dual_fr3_trunking_mtc.segment_executor import CONTROLLER_STATE_TOPICS


def test_readiness_requires_all_fourteen_arm_joints():
    expected = expected_arm_joint_names()

    assert len(expected) == 14
    assert "left_fr3_joint1" in expected
    assert "left_fr3_joint7" in expected
    assert "right_fr3_joint1" in expected
    assert "right_fr3_joint7" in expected
    assert missing_arm_joint_names(expected) == set()


def test_readiness_reports_only_missing_arm_joints():
    received = expected_arm_joint_names()
    received.remove("right_fr3_joint6")
    received.add("left_fr3_finger_joint1")

    assert missing_arm_joint_names(received) == {"right_fr3_joint6"}


def test_segment_executor_accepts_real_and_gazebo_controller_topics():
    assert CONTROLLER_STATE_TOPICS["left"] == (
        "/left/left_fr3_arm_controller/controller_state",
        "/left_fr3_arm_controller/controller_state",
    )
    assert CONTROLLER_STATE_TOPICS["right"] == (
        "/right/right_fr3_arm_controller/controller_state",
        "/right_fr3_arm_controller/controller_state",
    )


def test_readiness_selects_backend_specific_gripper_action():
    assert gripper_command_action_name("left", "fake") == (
        "/left_franka_gripper/gripper_action"
    )
    assert gripper_command_action_name("right", "gazebo") == (
        "/right_franka_gripper/gripper_cmd"
    )
