from __future__ import annotations

import argparse
import logging
from typing import Callable, Sequence

from ..gripper import GripperController, GripperProfileRegistry, GripperRequest
from ..models import TaskPlan
from ..stages.specs import MtcStageSpec
from .task_builder import create_mtc_task
from .planning import plan_with_retries


def execute_stage_by_stage(
    node,
    specs: Sequence[MtcStageSpec],
    task_plan: TaskPlan,
    args: argparse.Namespace,
    logger: logging.Logger,
    gripper_controller: GripperController | None = None,
    gripper_profiles: GripperProfileRegistry | None = None,
    confirmation_callback: Callable[[MtcStageSpec], bool] | None = None,
    task_factory: Callable = create_mtc_task,
    cable_controller=None,
) -> bool:
    """Execute preparation/interactive stages with bounded planning retries."""
    executable_specs = [spec for spec in specs if spec.executable]
    logger.info(
        "executing %d preparation/interactive stages with planning retries",
        len(executable_specs),
    )
    for ordinal, spec in enumerate(executable_specs, start=1):
        if getattr(spec, "confirmation_required", False):
            if confirmation_callback is None:
                logger.error(
                    "stage [%02d] requires operator confirmation, but no "
                    "confirmation callback is configured",
                    spec.stage_index,
                )
                return False
            if not confirmation_callback(spec):
                return False

        if getattr(spec, "mtc_stage_type", "") == "SimulationCable":
            try:
                if cable_controller is None or not cable_controller.execute(spec):
                    logger.error("simulation cable insertion failed; stopping before descent")
                    return False
            except Exception:
                logger.exception("simulation cable insertion raised; stopping before descent")
                return False
            continue

        if getattr(spec, "mtc_stage_type", "") == "GripperOperation":
            logger.info(
                "executing gripper stage %d/%d: [%02d] %s",
                ordinal,
                len(executable_specs),
                spec.stage_index,
                spec.name,
            )
            if gripper_controller is None:
                logger.error(
                    "gripper controller is unavailable for stage [%02d]",
                    spec.stage_index,
                )
                return False
            try:
                succeeded = gripper_controller.execute(
                    GripperRequest(
                        actor=spec.actor,
                        profile=spec.gripper_profile,
                        action_override=spec.gripper_action or None,
                        width_override=spec.gripper_width_override,
                    )
                )
            except Exception:  # noqa: BLE001 - execution must fail closed
                logger.exception(
                    "gripper stage [%02d] raised; halting all later stages",
                    spec.stage_index,
                )
                return False
            if not succeeded:
                logger.error(
                    "gripper stage [%02d] failed; halting all later stages",
                    spec.stage_index,
                )
                return False
            logger.info("gripper stage [%02d] completed", spec.stage_index)
            continue

        logger.info(
            "planning stage %d/%d: [%02d] %s",
            ordinal,
            len(executable_specs),
            spec.stage_index,
            spec.name,
        )
        planned = plan_with_retries(
            node, task_plan, specs, args, logger,
            gripper_profiles=gripper_profiles,
            selected_stage_indices={spec.stage_index},
            task_factory=task_factory,
        )
        if planned is None:
            logger.error(
                "planning failed for stage [%02d] %s; halting all later stages",
                spec.stage_index,
                spec.name,
            )
            return False
        try:
            result = planned.task.execute(planned.solution)
        except Exception:  # noqa: BLE001 - execution must fail closed
            logger.exception(
                "stage [%02d] raised while executing; halting all later stages",
                spec.stage_index,
            )
            return False
        if not result:
            logger.error(
                "execution failed for stage [%02d] %s (MoveIt error %s); "
                "halting all later stages",
                spec.stage_index,
                spec.name,
                getattr(result, "val", result),
            )
            return False
        logger.info("stage [%02d] completed", spec.stage_index)
    return True
