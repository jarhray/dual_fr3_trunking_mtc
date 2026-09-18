from __future__ import annotations

import argparse
import logging
import sys
import time
from functools import partial
from pathlib import Path
from typing import Sequence


from dual_fr3_trunking_mtc.execution.gripper import (
    GripperController,
    GripperProfileRegistry,
    resolve_gripper_backend,
)
from dual_fr3_trunking_mtc.task.models import TaskPlan
from dual_fr3_trunking_mtc.runtime.console_logging import make_logger
from dual_fr3_trunking_mtc.mtc.executor import execute_stage_by_stage
from dual_fr3_trunking_mtc.mtc.cached_execution import execute_cached_solution
from dual_fr3_trunking_mtc.mtc.cartesian_validation import validate_cartesian_settings
from dual_fr3_trunking_mtc.mtc.planning import create_task_from_args, plan_with_retries
from dual_fr3_trunking_mtc.mtc.preparation_search import (
    plan_preparation_candidates,
    validate_preparation_search_settings,
)
from dual_fr3_trunking_mtc.mtc.task_builder import (
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
from dual_fr3_trunking_mtc.task.planner import build_segment_plans, load_keypoints
from dual_fr3_trunking_mtc.task.preparation import (
    PreparationConfig,
    confirm_stage,
    read_controlling_terminal,
)
from dual_fr3_trunking_mtc.runtime.config import DEFAULTS
from dual_fr3_trunking_mtc.runtime.arguments import (
    _default_keypoints_file, _default_gripper_profiles_file, _parse_bool, _parse_args,
)
from dual_fr3_trunking_mtc.runtime.stage_publisher import (
    shutdown_stage_sequence_publisher,
    spin_stage_sequence_publisher,
    start_stage_sequence_publisher,
)
from dual_fr3_trunking_mtc.task.scheduler import build_task_plan, build_task_schedule
from dual_fr3_trunking_mtc.stages.compiler import build_mtc_stage_specs
from dual_fr3_trunking_mtc.stages.specs import (
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
    return make_logger()


def _execute_stage_by_stage(
    node,
    specs: Sequence[MtcStageSpec],
    task_plan: TaskPlan,
    args: argparse.Namespace,
    logger: logging.Logger,
    gripper_controller: GripperController | None = None,
    gripper_profiles: GripperProfileRegistry | None = None,
    confirmation_callback=None,
    cable_controller=None,
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
        cable_controller=cable_controller,
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
    cable_controller = None
    terminal_insertion = None
    try:
        from dual_fr3_trunking_mtc.execution.simulation_cable import (
            preparation_cable_config,
            SimulationCableController,
        )
        cable_config = preparation_cable_config(args.simulation_backend, args.maniskill_cable,
                                                args.preparation_enabled, args.cable_config)
        if args.simulation_backend == "maniskill" and not args.load_cable and not cable_config:
            raise ValueError("USB-only preparation requires maniskill_cable=true and preparation_enabled=true")
        if args.insertion_enabled and (not cable_config or args.simulation_backend != "maniskill"):
            raise ValueError("Insertion requires ManiSkill contact-grasp preparation")
        gripper_profiles = GripperProfileRegistry.load(
            args.gripper_profiles_file
        )
        gripper_backend = resolve_gripper_backend(args.simulation_backend)
        logger.info(
            "gripper backend=%s, profiles=%s",
            gripper_backend,
            ", ".join(gripper_profiles.names()),
        )
        keypoints = load_keypoints(args.keypoints_file, fallback_frame=args.task_frame)
        logger.info("keypoints_file=%s", Path(args.keypoints_file).expanduser().resolve())
        for index, keypoint in enumerate(keypoints):
            logger.info(
                "keypoint[%d] %s position=%s m frame=%s in_slot=%s role=%s",
                index, keypoint.name, keypoint.position, keypoint.frame_id,
                keypoint.in_slot, keypoint.role,
            )
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
            simulation_cable_config=cable_config,
            load_cable=args.load_cable,
            tool_roll=args.tool_roll,
            tool_pitch=args.tool_pitch,
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
                use_sim_time=gripper_backend in ("gazebo", "maniskill"),
            )
        if cable_config and (args.execute or (args.plan and args.insertion_enabled)):
            cable_controller = SimulationCableController(backend=args.simulation_backend)
            if args.insertion_enabled:
                from dual_fr3_trunking_mtc.insertion_task.pipeline import TerminalInsertion
                terminal_insertion = TerminalInsertion(cable_controller, cable_config, node, logger)
                terminal_insertion.sync_socket()

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
        if cable_config and not args.load_cable:
            logger.info("USB-only trajectory mode: cable physics disabled; original "
                        "dual-arm motion stages retained for USB transport testing, not cable-routing validation")
        if args.plan:
            validate_continuation = (
                terminal_insertion.validate_continuation if terminal_insertion is not None else None)
            remaining_planner = partial(plan_with_retries, solution_validator=validate_continuation)
            if preparation_specs:
                planned = plan_preparation_candidates(
                    node, task_plan, specs, args, logger, gripper_profiles=gripper_profiles,
                    solution_validator=validate_continuation,
                )
            else:
                planned = plan_with_retries(
                    node, task_plan, specs, args, logger, gripper_profiles=gripper_profiles,
                    solution_validator=validate_continuation,
                )
            if planned is None:
                return 2
            if args.publish_solution:
                planned.task.publish(planned.solution)
            if args.execute and args.execute_stage_by_stage:
                if not execute_cached_solution(
                    planned, node, task_plan, args, logger,
                    gripper_controller=gripper_controller,
                    cable_controller=cable_controller,
                    gripper_profiles=gripper_profiles,
                    planner=remaining_planner,
                    replan_after_grasp=terminal_insertion is not None,
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

        if cable_config and not args.load_cable:
            logger.info("USB-only trajectory debugging: original MTC motion stages retained; "
                        "no cable exists, so this is not a cable-routing completion result")
        if args.insertion_enabled and args.run_insertion and args.plan and args.execute:
            if not terminal_insertion.execute(gripper_controller):
                return 4
        stage_publisher = _start_stage_sequence_publisher(specs, logger)
        if args.keep_alive_sec > 0.0:
            deadline = time.monotonic() + args.keep_alive_sec
            while time.monotonic() < deadline:
                _spin_stage_sequence_publisher(stage_publisher, timeout_sec=0.1)
        return 0
    finally:
        _shutdown_stage_sequence_publisher(stage_publisher)
        if cable_controller is not None:
            cable_controller.close()
        if gripper_controller is not None:
            gripper_controller.close()
        rclcpp.shutdown()
