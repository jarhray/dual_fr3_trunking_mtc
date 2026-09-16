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


@pytest.mark.parametrize("backend,gripper_backend", [
    ("real", "franka"), ("fake", "fake"), ("gazebo", "gazebo"), ("maniskill", "maniskill"),
])
def test_simulation_backend_selects_the_gripper_interface(backend, gripper_backend):
    assert resolve_gripper_backend(backend) == gripper_backend


def test_backend_specific_command_names_are_not_probed_at_runtime():
    assert gripper_command_action_name("left", "fake") == (
        "/left_franka_gripper/gripper_action"
    )
    assert gripper_command_action_name("right", "gazebo") == (
        "/right_franka_gripper/gripper_cmd"
    )
    assert gripper_command_action_name("left", "maniskill") == "/left_franka_gripper/gripper_cmd"


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


@pytest.mark.parametrize("clock_rate,complete_wall,expected_done", [
    (.02, 11., True),  # 11 wall seconds are only .22 simulation seconds.
    (1., 11., False),  # Profile still expires at 10 simulation seconds.
    (0., 301., False),  # Paused simulation cannot wait forever.
])
def test_maniskill_result_timeout_uses_simulation_clock_with_wall_watchdog(
        monkeypatch, clock_rate, complete_wall, expected_done):
    from types import SimpleNamespace as NS
    from dual_fr3_trunking_mtc.gripper import GripperController
    wall = [0.]
    monkeypatch.setattr('dual_fr3_trunking_mtc.gripper.time.monotonic', lambda: wall[0])
    controller = object.__new__(GripperController)
    controller.backend = 'maniskill'
    controller._context = NS(ok=lambda: True)
    controller._executor = None
    controller.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(wall[0]*clock_rate*1.e9))))
    def spin(*args, **kwargs):
        wall[0] += 1.
    controller._rclpy = NS(spin_once=spin)
    future = NS(done=lambda: wall[0] >= complete_wall)
    controller._wait_for_result(future, 10.)
    assert future.done() is expected_done
    assert wall[0] <= 300.
