"""MTC command-line parsing and validation; numeric defaults live in config.py."""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

from ament_index_python.packages import get_package_share_directory
from rclpy.utilities import remove_ros_args

from ..task.models import ORIENTATION_DIRECTIONS
from ..mtc.cartesian_validation import validate_cartesian_settings
from ..mtc.preparation_search import validate_preparation_search_settings
from .config import DEFAULTS, SIMULATION_BACKENDS, parse_bool


def _default_keypoints_file() -> str:
    return str(
        Path(get_package_share_directory("dual_fr3_trunking_mtc"))
        / "config"
        / "keypoints.yaml"
    )


def _default_gripper_profiles_file() -> str:
    return str(
        Path(get_package_share_directory("dual_fr3_trunking_mtc"))
        / "config"
        / "gripper_profiles.yaml"
    )


def _parse_bool(value: str | bool) -> bool:
    return parse_bool(value)


def _parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--keypoints-file", default=_default_keypoints_file())
    parser.add_argument("--task-frame", default=DEFAULTS.task_frame)
    parser.add_argument(
        "--initial-leader-index",
        type=int,
        default=DEFAULTS.initial_leader_index,
    )
    parser.add_argument(
        "--initial-follower-index",
        type=int,
        default=DEFAULTS.initial_follower_index,
    )
    parser.add_argument("--leader-group", default=DEFAULTS.leader_group)
    parser.add_argument("--follower-group", default=DEFAULTS.follower_group)
    parser.add_argument("--leader-ik-frame", default=DEFAULTS.leader_ik_frame)
    parser.add_argument("--follower-ik-frame", default=DEFAULTS.follower_ik_frame)
    parser.add_argument(
        "--leader-orientation-direction",
        choices=ORIENTATION_DIRECTIONS,
        default=DEFAULTS.leader_orientation_direction,
    )
    parser.add_argument(
        "--follower-orientation-direction",
        choices=ORIENTATION_DIRECTIONS,
        default=DEFAULTS.follower_orientation_direction,
    )
    parser.add_argument(
        "--cartesian-step-size",
        type=float,
        default=DEFAULTS.cartesian_step_size,
    )
    parser.add_argument(
        "--cartesian-jump-threshold",
        type=float,
        default=DEFAULTS.cartesian_jump_threshold,
        help="relative joint-space jump factor (> 1); requires the full Cartesian path",
    )
    parser.add_argument(
        "--cartesian-path-tolerance",
        type=float,
        default=DEFAULTS.cartesian_path_tolerance,
        help="maximum TCP deviation from a line or in-place turn position, in meters",
    )
    parser.add_argument(
        "--motion-velocity-scaling",
        type=float,
        default=DEFAULTS.motion_velocity_scaling,
    )
    parser.add_argument(
        "--motion-acceleration-scaling",
        type=float,
        default=DEFAULTS.motion_acceleration_scaling,
    )
    parser.add_argument(
        "--anchor-max-path-z",
        type=float,
        default=DEFAULTS.anchor_max_path_z,
        help=(
            "maximum TCP z during direct_move_to_next_anchor, measured in "
            "the keypoint frame"
        ),
    )
    parser.add_argument(
        "--anchor-max-path-length-ratio",
        type=float,
        default=DEFAULTS.anchor_max_path_length_ratio,
        help="maximum anchor TCP path length / actual start-to-target distance (>= 1)",
    )
    parser.add_argument(
        "--leader-lead-distance",
        type=float,
        default=DEFAULTS.leader_lead_distance,
    )
    parser.add_argument("--tool-roll", type=float, default=DEFAULTS.tool_roll)
    parser.add_argument("--tool-pitch", type=float, default=DEFAULTS.tool_pitch)
    parser.add_argument(
        "--preparation-enabled",
        type=_parse_bool,
        default=DEFAULTS.preparation_enabled,
    )
    parser.add_argument(
        "--preparation-height",
        type=float,
        default=DEFAULTS.preparation_height,
    )
    parser.add_argument(
        "--preparation-interactive",
        type=_parse_bool,
        default=DEFAULTS.preparation_interactive,
    )
    for name, kind, help_text in (
        ("preparation_ik_candidates", int, "maximum distinct preparation IK candidates per arm"),
        ("preparation_ik_attempts", int, "maximum IK seeds tried per arm"),
        ("preparation_ik_timeout", float, "maximum seconds per preparation IK seed"),
        ("preparation_min_joint_distance", float, "minimum IK candidate joint distance (rad)"),
        ("preparation_candidate_attempts", int, "maximum rounds across preparation pairs"),
        ("preparation_search_timeout", float, "search seconds, checked between solver calls"),
    ):
        parser.add_argument(
            "--" + name.replace("_", "-"), type=kind, default=getattr(DEFAULTS, name),
            help=help_text,
        )
    parser.add_argument(
        "--gripper-profiles-file",
        default=_default_gripper_profiles_file(),
    )
    parser.add_argument(
        "--preparation-leader-gripper-profile",
        default=DEFAULTS.preparation_leader_gripper_profile,
    )
    parser.add_argument(
        "--preparation-follower-gripper-profile",
        default=DEFAULTS.preparation_follower_gripper_profile,
    )
    parser.add_argument("--plan", type=_parse_bool, default=DEFAULTS.plan)
    parser.add_argument("--simulation-backend", default=DEFAULTS.simulation_backend,
                        choices=SIMULATION_BACKENDS)
    parser.add_argument("--maniskill-cable", type=_parse_bool, default=True,
                        help="enable USB contact-grasp preparation with the maniskill backend")
    parser.add_argument("--insertion-enabled", type=_parse_bool, default=None)
    parser.add_argument('--run-insertion', type=_parse_bool, default=DEFAULTS.run_insertion,
                        help='false: finish transport and leave the enabled insertion scene for the independent skill')
    parser.add_argument("--load-cable", type=_parse_bool, default=DEFAULTS.load_cable,
                        help="false: disable cable physics, retaining dual-arm USB transport motions")
    parser.add_argument("--cable-config", default="")
    parser.add_argument("--execute", type=_parse_bool, default=DEFAULTS.execute)
    parser.add_argument(
        "--planning-attempts",
        type=int,
        default=DEFAULTS.planning_attempts,
        help="planning attempts, also used for pipeline failures within a preparation pair",
    )
    parser.add_argument(
        "--execution-replan-attempts",
        type=int,
        default=DEFAULTS.execution_replan_attempts,
        help="maximum replans of unfinished stages after terminal execution failures",
    )
    parser.add_argument(
        "--replan-after-grasp", type=_parse_bool, default=DEFAULTS.replan_after_grasp,
        help="allow replanning only if cached trajectories fail measured-scene validation after grasp",
    )
    parser.add_argument(
        "--execute-stage-by-stage",
        type=_parse_bool,
        default=DEFAULTS.execute_stage_by_stage,
        help=(
            "execute stages from the same successful full solution; replan only "
            "after execution failure or explicitly enabled post-grasp validation failure. "
            "False requires no preparation/gripper stages "
            "and sends one whole-task action without automatic recovery"
        ),
    )
    parser.add_argument(
        "--publish-solution",
        type=_parse_bool,
        default=DEFAULTS.publish_solution,
    )
    parser.add_argument(
        "--keep-alive-sec",
        type=float,
        default=DEFAULTS.keep_alive_sec,
    )
    args = parser.parse_args(remove_ros_args(args=[parser.prog, *argv])[1:])
    if args.insertion_enabled is None:
        args.insertion_enabled = (
            DEFAULTS.insertion_enabled and args.simulation_backend == 'maniskill' and args.maniskill_cable)
    try:
        validate_cartesian_settings(args.cartesian_jump_threshold, args.cartesian_path_tolerance)
        validate_preparation_search_settings(args)
    except ValueError as exc:
        parser.error(str(exc))
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be >= 1")
    if args.execution_replan_attempts < 0:
        parser.error("--execution-replan-attempts must be >= 0")
    return args
