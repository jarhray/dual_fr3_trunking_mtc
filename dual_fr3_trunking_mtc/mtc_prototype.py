from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Iterable, Sequence

from ament_index_python.packages import get_package_share_directory

from .gripper import (
    GripperController,
    GripperProfileRegistry,
    resolve_gripper_backend,
)
from .models import ORIENTATION_DIRECTIONS, TaskPlan
from .mtc.executor import execute_stage_by_stage
from .mtc.cached_execution import execute_cached_solution
from .mtc.cartesian_validation import validate_cartesian_settings
from .mtc.planning import create_task_from_args, plan_with_retries
from .mtc.preparation_search import (
    plan_preparation_candidates,
    validate_preparation_search_settings,
)
from .mtc.task_builder import (
    ANCHOR_PATH_CONSTRAINT_MIN_Z,
    ANCHOR_PATH_CONSTRAINT_XY_SIZE,
    DEFAULT_ANCHOR_MAX_PATH_Z,
    OMPL_MOVE_TO_TIMEOUT,
    OMPL_NUM_PLANNING_ATTEMPTS,
    OMPL_PIPELINE_NAME,
    OMPL_PLANNER_ID,
    create_motion_planners,
    create_mtc_task,
    import_mtc_modules,
    max_link_z_path_constraint,
    planner_for_move_to,
)
from .planner import build_segment_plans, load_keypoints
from .preparation import (
    PreparationConfig,
    confirm_stage,
    read_controlling_terminal,
)
from .runtime.config import DEFAULTS, parse_bool
from .runtime.stage_publisher import (
    shutdown_stage_sequence_publisher,
    spin_stage_sequence_publisher,
    start_stage_sequence_publisher,
)
from .scheduler import build_task_plan, build_task_schedule
from .stages.compiler import build_mtc_stage_specs
from .stages.specs import (
    MtcStageSpec,
    mtc_stage_sequence_to_dict,
    mtc_stage_sequence_to_json,
    mtc_stage_sequence_to_text,
    mtc_stage_spec_to_dict,
)


__all__ = [
    "ANCHOR_PATH_CONSTRAINT_MIN_Z",
    "ANCHOR_PATH_CONSTRAINT_XY_SIZE",
    "DEFAULT_ANCHOR_MAX_PATH_Z",
    "MtcStageSpec",
    "OMPL_MOVE_TO_TIMEOUT",
    "OMPL_NUM_PLANNING_ATTEMPTS",
    "OMPL_PIPELINE_NAME",
    "OMPL_PLANNER_ID",
    "_create_motion_planners",
    "_execute_stage_by_stage",
    "_import_mtc_modules",
    "_max_link_z_path_constraint",
    "_planner_for_move_to",
    "build_mtc_stage_specs",
    "build_task_plan",
    "build_task_schedule",
    "create_mtc_task",
    "main",
    "mtc_stage_sequence_to_dict",
    "mtc_stage_sequence_to_json",
    "mtc_stage_sequence_to_text",
    "mtc_stage_spec_to_dict",
]


# Compatibility aliases for callers and tests that used the original monolithic
# module. New code should import these helpers from stages/, mtc/, or runtime/.
_import_mtc_modules = import_mtc_modules
_create_motion_planners = create_motion_planners
_planner_for_move_to = planner_for_move_to
_max_link_z_path_constraint = max_link_z_path_constraint
_start_stage_sequence_publisher = start_stage_sequence_publisher
_spin_stage_sequence_publisher = spin_stage_sequence_publisher
_shutdown_stage_sequence_publisher = shutdown_stage_sequence_publisher


def _make_logger() -> logging.Logger:
    logging.basicConfig(
        level=logging.INFO,
        format="[mtc_prototype] %(message)s",
        stream=sys.stdout,
    )
    return logging.getLogger("dual_fr3_trunking_mtc.mtc_prototype")


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
    parser.add_argument(
        "--use-fake-hardware",
        type=_parse_bool,
        default=DEFAULTS.use_fake_hardware,
    )
    parser.add_argument(
        "--use-gazebo",
        type=_parse_bool,
        default=DEFAULTS.use_gazebo,
    )
    parser.add_argument("--plan", type=_parse_bool, default=DEFAULTS.plan)
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
        "--execute-stage-by-stage",
        type=_parse_bool,
        default=DEFAULTS.execute_stage_by_stage,
        help=(
            "execute stages from the same successful full solution; replan only "
            "after execution failure. False requires no preparation/gripper stages "
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
    args, _unknown = parser.parse_known_args(list(argv))
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


def _execute_stage_by_stage(
    node,
    specs: Sequence[MtcStageSpec],
    task_plan: TaskPlan,
    args: argparse.Namespace,
    logger: logging.Logger,
    gripper_controller: GripperController | None = None,
    gripper_profiles: GripperProfileRegistry | None = None,
    confirmation_callback=None,
) -> bool:
    """Compatibility wrapper that keeps create_mtc_task monkeypatchable here."""
    return execute_stage_by_stage(
        node,
        specs,
        task_plan,
        args,
        logger,
        gripper_controller=gripper_controller,
        gripper_profiles=gripper_profiles,
        confirmation_callback=confirmation_callback,
        task_factory=create_mtc_task,
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    logger = _make_logger()
    rclcpp, _core, _stages = _import_mtc_modules()

    rclcpp.init()
    node_options = rclcpp.NodeOptions(
        automatically_declare_parameters_from_overrides=True,
    )
    node = rclcpp.Node("dual_fr3_trunking_mtc_prototype", node_options)
    stage_publisher = None
    gripper_controller = None
    try:
        gripper_profiles = GripperProfileRegistry.load(
            args.gripper_profiles_file
        )
        gripper_backend = resolve_gripper_backend(
            use_fake_hardware=args.use_fake_hardware,
            use_gazebo=args.use_gazebo,
        )
        logger.info(
            "gripper backend=%s, profiles=%s",
            gripper_backend,
            ", ".join(gripper_profiles.names()),
        )
        keypoints = load_keypoints(args.keypoints_file, fallback_frame=args.task_frame)
        segments = build_segment_plans(
            keypoints,
            leader_orientation_direction=args.leader_orientation_direction,
            follower_orientation_direction=args.follower_orientation_direction,
        )
        task_plan = build_task_plan(
            segments,
            initial_leader_index=args.initial_leader_index,
            initial_follower_index=args.initial_follower_index,
        )
        preparation_config = PreparationConfig(
            approach_height=args.preparation_height,
            leader_group=args.leader_group,
            follower_group=args.follower_group,
            leader_ik_frame=args.leader_ik_frame,
            follower_ik_frame=args.follower_ik_frame,
            leader_orientation_direction=args.leader_orientation_direction,
            follower_orientation_direction=args.follower_orientation_direction,
            interactive=args.preparation_interactive,
            leader_gripper_profile=args.preparation_leader_gripper_profile,
            follower_gripper_profile=args.preparation_follower_gripper_profile,
        )
        # Validate preparation profile names even in planning-only mode.
        gripper_profiles.get(preparation_config.leader_gripper_profile)
        gripper_profiles.get(preparation_config.follower_gripper_profile)
        specs = build_mtc_stage_specs(
            task_plan,
            leader_group=args.leader_group,
            follower_group=args.follower_group,
            leader_ik_frame=args.leader_ik_frame,
            follower_ik_frame=args.follower_ik_frame,
            leader_lead_distance=args.leader_lead_distance,
            leader_orientation_direction=args.leader_orientation_direction,
            follower_orientation_direction=args.follower_orientation_direction,
            preparation_config=(
                preparation_config if args.preparation_enabled else None
            ),
        )
        preparation_specs = [
            spec for spec in specs if spec.phase == "preparation"
        ]
        formal_specs = [spec for spec in specs if spec.phase == "formal"]
        formal_gripper_specs = [
            spec for spec in formal_specs
            if spec.mtc_stage_type == "GripperOperation"
        ]
        if (args.execute and (formal_gripper_specs or preparation_specs)
                and not args.execute_stage_by_stage):
            logger.error(
                "preparation confirmations and profile-based gripper operations require "
                "execute_stage_by_stage=true"
            )
            return 2
        if args.execute and (
            args.preparation_enabled or formal_gripper_specs
        ):
            gripper_controller = GripperController(
                gripper_profiles,
                gripper_backend,
            )

        logger.info(
            "loaded %d keypoints, %d segments, %d task steps, "
            "%d preparation stages and %d formal MTC stages",
            len(task_plan.keypoints),
            len(task_plan.segments),
            len(task_plan.steps),
            len(preparation_specs),
            len(formal_specs),
        )
        logger.info(
            "anchor OMPL path constraint: TCP z <= %.3f m in the keypoint frame",
            args.anchor_max_path_z,
        )
        logger.info(
            "anchor TCP path length <= %.3f x actual start-to-target distance",
            args.anchor_max_path_length_ratio,
        )
        logger.info(
            "unified MTC stage sequence:\n%s",
            mtc_stage_sequence_to_text(specs),
        )
        if args.plan:
            if preparation_specs:
                planned = plan_preparation_candidates(
                    node, task_plan, specs, args, logger, gripper_profiles=gripper_profiles,
                )
            else:
                planned = plan_with_retries(
                    node, task_plan, specs, args, logger, gripper_profiles=gripper_profiles,
                )
            if planned is None:
                return 2
            if args.publish_solution:
                planned.task.publish(planned.solution)
            if args.execute and args.execute_stage_by_stage:
                if not execute_cached_solution(
                    planned, node, task_plan, args, logger,
                    gripper_controller=gripper_controller,
                    gripper_profiles=gripper_profiles,
                    confirmation_callback=lambda spec: confirm_stage(
                        spec, read_controlling_terminal, logger,
                    ),
                ):
                    return 3
            elif args.execute:
                # A single action has no failed-stage information in Task.execute's
                # return value. Never restart an unknown, partially executed task.
                logger.info("executing the selected whole-task solution directly")
                try:
                    result = planned.task.execute(planned.solution)
                except Exception:
                    logger.exception("whole-task execution raised; stopping")
                    return 3
                if not result:
                    logger.error(
                        "whole-task execution failed (MoveIt error %s); progress "
                        "is unknown, stopping. Use execute_stage_by_stage=true "
                        "for recovery with cached trajectories",
                        getattr(result, "val", result),
                    )
                    return 3
                logger.info("whole-task solution execution finished")
        else:
            create_task_from_args(
                node, task_plan, specs, args, gripper_profiles=gripper_profiles,
            )
            logger.info("build-only mode; planning skipped")

        stage_publisher = _start_stage_sequence_publisher(specs, logger)
        if args.keep_alive_sec > 0.0:
            deadline = time.monotonic() + args.keep_alive_sec
            while time.monotonic() < deadline:
                _spin_stage_sequence_publisher(stage_publisher, timeout_sec=0.1)
        return 0
    finally:
        _shutdown_stage_sequence_publisher(stage_publisher)
        if gripper_controller is not None:
            gripper_controller.close()
        rclcpp.shutdown()
