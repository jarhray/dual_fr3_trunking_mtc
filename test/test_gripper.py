from pathlib import Path

import pytest

from dual_fr3_trunking_mtc.gripper import (
    GripperProfileRegistry,
    GripperRequest,
    gripper_command_action_name,
    gripper_result_succeeded,
    resolve_gripper_backend,
)


PROFILE_FILE = (
    Path(__file__).parents[1]
    / "config"
    / "gripper_profiles.yaml"
)


def test_launch_flags_resolve_to_one_explicit_backend():
    assert resolve_gripper_backend(
        use_fake_hardware=False,
        use_gazebo=False,
    ) == "franka"
    assert resolve_gripper_backend(
        use_fake_hardware=True,
        use_gazebo=False,
    ) == "fake"
    assert resolve_gripper_backend(
        use_fake_hardware=False,
        use_gazebo=True,
    ) == "gazebo"
    assert resolve_gripper_backend(
        use_fake_hardware=True,
        use_gazebo=True,
    ) == "gazebo"


def test_backend_specific_command_names_are_not_probed_at_runtime():
    assert gripper_command_action_name("left", "fake") == (
        "/left_franka_gripper/gripper_action"
    )
    assert gripper_command_action_name("right", "gazebo") == (
        "/right_franka_gripper/gripper_cmd"
    )


def test_profiles_use_total_width_and_convert_only_at_moveit_boundary():
    profiles = GripperProfileRegistry.load(PROFILE_FILE)
    profile = profiles.resolve(
        GripperRequest(actor="follower", profile="cable_tip")
    )

    assert profile.action == "grasp"
    assert profile.force > 0.0
    assert profile.finger_joint_position == pytest.approx(profile.width / 2.0)


def test_per_stage_width_override_still_obeys_global_safety_limits():
    profiles = GripperProfileRegistry.load(PROFILE_FILE)

    resolved = profiles.resolve(
        GripperRequest(
            actor="leader",
            profile="trunk_edge",
            width_override=0.03,
        )
    )
    assert resolved.width == pytest.approx(0.03)

    with pytest.raises(ValueError, match="outside safety limits"):
        profiles.resolve(
            GripperRequest(
                actor="leader",
                profile="trunk_edge",
                width_override=0.09,
            )
        )


@pytest.mark.parametrize(
    ("action", "reached_goal", "stalled", "expected"),
    [
        ("move", True, False, True),
        ("move", False, True, False),
        ("grasp", False, True, True),
        ("grasp", False, False, False),
    ],
)
def test_gripper_command_contact_counts_only_as_a_successful_grasp(
    action,
    reached_goal,
    stalled,
    expected,
):
    result = type(
        "Result",
        (),
        {"reached_goal": reached_goal, "stalled": stalled},
    )()

    assert gripper_result_succeeded(result, action) is expected
