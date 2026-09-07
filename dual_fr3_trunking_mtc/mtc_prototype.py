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
        "--execute-stage-by-stage",
        type=_parse_bool,
        default=DEFAULTS.execute_stage_by_stage,
        help=(
            "replan from the real robot state before each executable stage; "
            "disable only for legacy whole-solution execution"
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
        task, specs = create_mtc_task(
            node,
            task_plan,
            specs,
            leader_group=args.leader_group,
            follower_group=args.follower_group,
            leader_ik_frame=args.leader_ik_frame,
            follower_ik_frame=args.follower_ik_frame,
            cartesian_step_size=args.cartesian_step_size,
            motion_velocity_scaling=args.motion_velocity_scaling,
            motion_acceleration_scaling=args.motion_acceleration_scaling,
            leader_lead_distance=args.leader_lead_distance,
            tool_roll=args.tool_roll,
            tool_pitch=args.tool_pitch,
            leader_orientation_direction=args.leader_orientation_direction,
            follower_orientation_direction=args.follower_orientation_direction,
            anchor_max_path_z=args.anchor_max_path_z,
            gripper_profiles=gripper_profiles,
        )

        formal_gripper_specs = [
            spec for spec in formal_specs
            if spec.mtc_stage_type == "GripperOperation"
        ]
        if args.execute and formal_gripper_specs and not args.execute_stage_by_stage:
            logger.error(
                "profile-based seat_edge gripper operations require "
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
            "unified MTC stage sequence:\n%s",
            mtc_stage_sequence_to_text(specs),
        )
        if args.plan:
            if args.execute and preparation_specs:
                if not _execute_stage_by_stage(
                    node,
                    preparation_specs,
                    task_plan,
                    args,
                    logger,
                    gripper_controller=gripper_controller,
                    gripper_profiles=gripper_profiles,
                    confirmation_callback=lambda spec: confirm_stage(
                        spec,
                        read_controlling_terminal,
                        logger,
                    ),
                ):
                    return 3
                logger.info("preparation completed; starting the formal task")
                task, _ = create_mtc_task(
                    node,
                    task_plan,
                    formal_specs,
                    leader_group=args.leader_group,
                    follower_group=args.follower_group,
                    leader_ik_frame=args.leader_ik_frame,
                    follower_ik_frame=args.follower_ik_frame,
                    cartesian_step_size=args.cartesian_step_size,
                    motion_velocity_scaling=args.motion_velocity_scaling,
                    motion_acceleration_scaling=args.motion_acceleration_scaling,
                    leader_lead_distance=args.leader_lead_distance,
                    tool_roll=args.tool_roll,
                    tool_pitch=args.tool_pitch,
                    leader_orientation_direction=(
                        args.leader_orientation_direction
                    ),
                    follower_orientation_direction=(
                        args.follower_orientation_direction
                    ),
                    anchor_max_path_z=args.anchor_max_path_z,
                    gripper_profiles=gripper_profiles,
                )
            full_plan_succeeded = task.plan()
            if full_plan_succeeded:
                logger.info(
                    "planning succeeded with %d solutions",
                    len(task.solutions),
                )
                if args.publish_solution and task.solutions:
                    task.publish(task.solutions[0])
            elif not (args.execute and args.execute_stage_by_stage):
                logger.error("planning failed")
                return 2
            else:
                logger.warning(
                    "whole-chain planning failed; trying staged execution "
                    "from fresh robot states"
                )

            if args.execute and args.execute_stage_by_stage:
                if not _execute_stage_by_stage(
                    node,
                    formal_specs,
                    task_plan,
                    args,
                    logger,
                    gripper_controller=gripper_controller,
                    gripper_profiles=gripper_profiles,
                ):
                    return 3
                logger.info("staged MTC execution finished")
            elif args.execute and full_plan_succeeded and task.solutions:
                logger.warning(
                    "executing the whole MTC solution without real-state "
                    "replanning; this is legacy collision-prone behavior"
                )
                result = task.execute(task.solutions[0])
                if not result:
                    logger.error(
                        "whole-solution MTC execution failed (MoveIt error %s)",
                        getattr(result, "val", result),
                    )
                    return 3
                logger.info("MTC execution request finished")
        else:
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
