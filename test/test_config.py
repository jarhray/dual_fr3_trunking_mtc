import pytest

from dual_fr3_trunking_mtc.mtc.task_builder import DEFAULT_ANCHOR_MAX_PATH_Z
from dual_fr3_trunking_mtc.mtc_prototype import _parse_args
from dual_fr3_trunking_mtc.preparation import PreparationConfig
from dual_fr3_trunking_mtc.runtime.config import DEFAULTS, launch_default


def test_cli_defaults_are_sourced_from_runtime_config():
    args = _parse_args([])

    assert args.task_frame == DEFAULTS.task_frame
    assert args.initial_leader_index == DEFAULTS.initial_leader_index
    assert args.initial_follower_index == DEFAULTS.initial_follower_index
    assert args.anchor_max_path_z == pytest.approx(DEFAULTS.anchor_max_path_z)
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


def test_launch_default_preserves_typed_values():
    assert launch_default(True) == "true"
    assert launch_default(False) == "false"
    assert launch_default(0.4) == "0.4"
    assert launch_default("left_fr3_link0") == "left_fr3_link0"


def test_gazebo_resource_default_matches_moveit_launch():
    assert DEFAULTS.gazebo_effort is False
