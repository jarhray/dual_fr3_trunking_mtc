import pytest

from dual_fr3_trunking_mtc.mtc.task_builder import DEFAULT_ANCHOR_MAX_PATH_Z
from dual_fr3_trunking_mtc.mtc_prototype import _parse_args
from dual_fr3_trunking_mtc.preparation import PreparationConfig
from dual_fr3_trunking_mtc.runtime.config import DEFAULTS, launch_default, resolve_simulation_backend


def test_cli_defaults_are_sourced_from_runtime_config():
    args = _parse_args([])

    assert args.simulation_backend == DEFAULTS.simulation_backend == "maniskill"
    assert args.insertion_enabled and args.load_cable and args.execute
    assert DEFAULTS.cable_solver == 'rope_actor'
    assert not hasattr(args, "use_fake_hardware")
    assert not hasattr(args, "use_gazebo")
    assert args.task_frame == DEFAULTS.task_frame
    assert args.initial_leader_index == DEFAULTS.initial_leader_index
    assert args.initial_follower_index == DEFAULTS.initial_follower_index
    assert args.anchor_max_path_z == pytest.approx(DEFAULTS.anchor_max_path_z)
    assert args.anchor_max_path_length_ratio == pytest.approx(1.5)
    assert args.planning_attempts == DEFAULTS.planning_attempts
    assert args.execution_replan_attempts == 2
    assert args.cartesian_jump_threshold == pytest.approx(DEFAULTS.cartesian_jump_threshold)
    assert args.cartesian_path_tolerance == pytest.approx(DEFAULTS.cartesian_path_tolerance)
    assert args.preparation_ik_candidates == DEFAULTS.preparation_ik_candidates
    assert args.preparation_search_timeout == DEFAULTS.preparation_search_timeout
    assert args.preparation_height == pytest.approx(DEFAULTS.preparation_height)
    assert (
        args.preparation_follower_gripper_profile
        == DEFAULTS.preparation_follower_gripper_profile
    )
    assert args.keep_alive_sec == pytest.approx(DEFAULTS.keep_alive_sec)


def test_preparation_and_mtc_builder_share_runtime_defaults():
    preparation = PreparationConfig()

    assert preparation.approach_height == pytest.approx(
        DEFAULTS.preparation_height
    )
    assert (
        preparation.follower_gripper_profile
        == DEFAULTS.preparation_follower_gripper_profile
    )
    assert DEFAULT_ANCHOR_MAX_PATH_Z == pytest.approx(DEFAULTS.anchor_max_path_z)


def test_cli_accepts_path_length_ratio_override():
    args = _parse_args(["--anchor-max-path-length-ratio", "1.2"])
    assert args.anchor_max_path_length_ratio == 1.2


@pytest.mark.parametrize("option,value", [
    ("--planning-attempts", "0"), ("--execution-replan-attempts", "-1"),
    ("--cartesian-jump-threshold", "0"), ("--cartesian-jump-threshold", "1"),
    ("--cartesian-jump-threshold", "nan"), ("--cartesian-jump-threshold", "inf"),
    ("--cartesian-path-tolerance", "0"), ("--cartesian-path-tolerance", "-0.1"),
    ("--cartesian-path-tolerance", "nan"), ("--cartesian-path-tolerance", "inf"),
])
def test_cli_rejects_invalid_retry_limits(option, value):
    with pytest.raises(SystemExit):
        _parse_args([option, value])


def test_launch_default_preserves_typed_values():
    assert launch_default(True) == "true"
    assert launch_default(False) == "false"
    assert launch_default(0.4) == "0.4"
    assert launch_default("left_fr3_link0") == "left_fr3_link0"


def test_gazebo_resource_default_matches_moveit_launch():
    assert DEFAULTS.gazebo_effort is False


@pytest.mark.parametrize("backend", ["gazebo", "maniskill", "fake", "real"])
def test_backend_selection_is_explicit(backend):
    assert resolve_simulation_backend(backend) == backend
    assert _parse_args(["--simulation-backend", backend]).simulation_backend == backend


@pytest.mark.parametrize("backend", ["auto", "missing", ""])
def test_backend_selection_rejects_invalid_values(backend):
    with pytest.raises(ValueError):
        resolve_simulation_backend(backend)
    with pytest.raises(SystemExit):
        _parse_args(["--simulation-backend", backend])


@pytest.mark.parametrize("option", ["--use-gazebo", "--use-fake-hardware"])
def test_removed_cli_selectors_are_rejected(option):
    with pytest.raises(SystemExit):
        _parse_args([option, "false"])


def test_cli_still_accepts_ros_arguments():
    args = _parse_args([
        "--simulation-backend", "maniskill", "--ros-args", "-p", "use_sim_time:=true",
    ])
    assert args.simulation_backend == "maniskill"
