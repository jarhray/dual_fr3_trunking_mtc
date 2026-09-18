"""Execute a selected whole-task solution; replan only its unfinished suffix."""

import copy
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes

from dual_fr3_trunking_mtc.execution.gripper import GripperRequest
from dual_fr3_trunking_mtc.runtime.config import DEFAULTS
from dual_fr3_trunking_mtc.mtc.planning import plan_with_retries


# Only terminal MoveIt failures with a known failed stage can be recovered.
# Cancellation, lost communication and exceptions must not restart motion.
RECOVERABLE_EXECUTION_ERRORS = {
    MoveItErrorCodes.INVALID_MOTION_PLAN,
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE,
    MoveItErrorCodes.CONTROL_FAILED,
    MoveItErrorCodes.TIMED_OUT,
}


@dataclass
class CachedStage:
    spec: object
    solution: object | None


def _solution_ids(message):
    return {
        item.info.id
        for item in (*message.sub_solution, *message.sub_trajectory)
        if item.info.id != 0
    }


def cache_selected_stages(planned):
    """
    Select stage solutions belonging to the chosen complete solution.

    Stage.solutions[0] alone is insufficient: independently cheapest stages
    need not be connected to each other. Match MTC introspection solution IDs.
    """
    introspection = planned.task.introspection()
    selected_ids = _solution_ids(planned.solution.toMsg(introspection))
    cached = []
    for spec in planned.specs:
        if not spec.executable:
            continue
        if spec.mtc_stage_type in ("GripperOperation", "SimulationCable"):
            cached.append(CachedStage(spec, None))
            continue
        matches = []
        for candidate in planned.task[spec.name].solutions:
            ids = _solution_ids(candidate.toMsg(introspection))
            if ids and ids.issubset(selected_ids):
                matches.append(candidate)
        if len(matches) != 1:
            raise ValueError(f"cannot identify selected solution for stage {spec.name!r}")
        cached.append(CachedStage(spec, matches[0]))
    return cached


def capture_relative_targets(cached):
    """Freeze planned absolute endpoints before any relative motion executes."""
    targets = {}
    for item in cached:
        spec = item.spec
        if spec.mtc_stage_type == "Merger":
            relatives = spec.children
        elif spec.mtc_stage_type == "MoveRelative":
            relatives = (spec,)
        else:
            continue
        scene = item.solution.end.scene
        state = copy.copy(scene.current_state)
        state.update()
        for relative in relatives:
            target = PoseStamped()
            target.header.frame_id = scene.planning_frame
            target.pose = state.get_pose(relative.ik_frame)
            targets[relative.name] = target
    return targets


def capture_anchor_joint_targets(cached):
    """Keep the selected elbow/wrist branch when refreshing a measured grasp.

    Replanning to a pose alone can select a different IK endpoint and destroy
    the already verified continuation. Other recovery paths remain unchanged.
    """
    targets = {}
    for item in cached:
        spec = item.spec
        if getattr(spec, 'primitive', '') not in (
                'direct_move_to_next_anchor', 'direct_move_to_seat_edge_keypoint'):
            continue
        scene = item.solution.end.scene
        names = scene.robot_model.get_joint_model_group(spec.group).active_joint_model_names
        joints = scene.current_state.joint_positions
        targets[spec.name] = {name: joints[name] for name in names}
    return targets


def execute_cached_solution(
    planned, node, task_plan, args, logger, gripper_controller=None,
    gripper_profiles=None, confirmation_callback=None,
    planner=plan_with_retries,
    cable_controller=None,
    replan_after_grasp=False,
):
    """
    Execute exact cached trajectories, with bounded recovery after failure.

    Each action contains the stage subsolution from the same successful full
    plan. This preserves motion geometry/timing while locating failures and
    allowing profile-based gripper actions between motion stages.
    """
    recovery_limit = args.execution_replan_attempts
    if recovery_limit < 0:
        raise ValueError("execution_replan_attempts must be >= 0")
    recoveries = 0
    fixed_targets = {}
    confirmed = set()
    fixed_preparation_goals = copy.deepcopy(planned.preparation_joint_goals)
    while True:
        try:
            cached = cache_selected_stages(planned)
            for name, target in capture_relative_targets(cached).items():
                # Preserve the original endpoint over multiple recoveries.
                fixed_targets.setdefault(name, target)
        except Exception:  # noqa: BLE001 - reject an unmapped solution before motion
            logger.exception("cannot prepare selected solution for execution")
            return False
        logger.info("executing %d cached stages from the successful plan", len(cached))
        for index, item in enumerate(cached):
            spec = item.spec
            if cable_controller is not None and (getattr(spec, "phase", "") == "formal" or getattr(spec, "primitive", "") == "dual_cartesian_descent"):
                try:
                    cable_controller.ensure_grasp()
                except Exception:
                    logger.exception("USB grasp is not stable; stopping before transport")
                    return False
            if getattr(spec, "confirmation_required", False) and spec.stage_index not in confirmed:
                if confirmation_callback is None or not confirmation_callback(spec):
                    logger.error(
                        "confirmation missing or declined for stage [%02d]", spec.stage_index,
                    )
                    return False
                confirmed.add(spec.stage_index)
                if cable_controller is not None:
                    try:
                        cable_controller.ensure_grasp()
                    except Exception:
                        logger.exception('USB grasp lost while waiting for confirmation; stopping')
                        return False
            logger.info("executing cached stage [%02d] %s", spec.stage_index, spec.name)
            if spec.mtc_stage_type == "SimulationCable":
                try:
                    if cable_controller is None or not cable_controller.execute(spec):
                        logger.error("simulation USB operation failed; stopping before transport")
                        return False
                except Exception:
                    logger.exception("simulation USB operation raised; stopping before transport")
                    return False
                if replan_after_grasp and spec.cable_operation == 'release_verify':
                    # release_verify has just installed the measured attachment.
                    # Replan the suffix before Enter, without repeating the grasp
                    # or shifting any already selected Cartesian endpoint.
                    remaining = tuple(entry.spec for entry in cached[index + 1:])
                    logger.info('grasp released and verified; planning the remaining path from measured grasp')
                    planned = planner(
                        node, task_plan, remaining, args, logger,
                        gripper_profiles=gripper_profiles, recovery_pose_goals=fixed_targets,
                        preparation_joint_goals=fixed_preparation_goals,
                        recovery_joint_goals=capture_anchor_joint_targets(cached[index + 1:]),
                    )
                    if planned is None:
                        return False
                    if getattr(args, 'publish_solution', DEFAULTS.publish_solution):
                        planned.task.publish(planned.solution)
                    break
                continue
            if spec.mtc_stage_type == "GripperOperation":
                try:
                    succeeded = gripper_controller is not None and gripper_controller.execute(
                        GripperRequest(
                            actor=spec.actor, profile=spec.gripper_profile,
                            action_override=spec.gripper_action or None,
                            width_override=spec.gripper_width_override,
                        )
                    )
                except Exception:  # noqa: BLE001 - grasp state is uncertain
                    logger.exception("gripper stage [%02d] raised", spec.stage_index)
                    return False
                if not succeeded:
                    logger.error("gripper stage [%02d] failed; stopping", spec.stage_index)
                    return False
                logger.info("gripper stage [%02d] completed", spec.stage_index)
                continue

            try:
                result = planned.task.execute(item.solution)
            except Exception:  # noqa: BLE001 - execution may still be active
                logger.exception(
                    "execution raised at stage [%02d]; motion state is uncertain, stopping",
                    spec.stage_index,
                )
                return False
            if result:
                logger.info("stage [%02d] completed", spec.stage_index)
                continue
            code = getattr(result, "val", None)
            if code not in RECOVERABLE_EXECUTION_ERRORS or recoveries >= recovery_limit:
                logger.error(
                    "stage [%02d] failed (MoveIt error %s); stopping after %d recoveries",
                    spec.stage_index, code, recoveries,
                )
                return False

            recoveries += 1
            remaining = tuple(entry.spec for entry in cached[index:])
            logger.warning(
                "stage [%02d] failed (MoveIt error %s); recovery %d/%d: "
                "replan %d unfinished stages from CurrentState to the original targets",
                spec.stage_index, code, recoveries, recovery_limit, len(remaining),
            )
            planned = planner(
                node, task_plan, remaining, args, logger,
                gripper_profiles=gripper_profiles, recovery_pose_goals=fixed_targets,
                preparation_joint_goals=fixed_preparation_goals,
            )
            if planned is None:
                return False
            if getattr(args, "publish_solution", DEFAULTS.publish_solution):
                planned.task.publish(planned.solution)
            break
        else:
            logger.info("cached MTC solution execution finished")
            return True
