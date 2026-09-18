"""Read-only, stage-scoped explanations for unsuccessful MTC planning attempts."""

from collections import Counter
import copy
from dataclasses import dataclass, field
import logging

import numpy as np


def append_failure_comment(solution, message):
    """Annotate a solution; callback bindings may pass a copy of the native one."""
    previous = solution.comment
    solution.comment = f"{previous}; {message}" if previous else message


def explain_cost_failure(solution, spec, task_plan, args):
    """Recover a lost cost comment on the stored *failed* solution.

    Some pybind versions copy the const SubTrajectory passed to a cost callback.
    Re-evaluate that check only for uncommented failures, using the original
    settings. This updates diagnostic text, never the cost or acceptance result.
    """
    if solution.comment or args is None or getattr(solution, "trajectory", None) is None:
        return
    from dual_fr3_trunking_mtc.mtc.cartesian_validation import (
        cartesian_path_cost,
        merged_cartesian_path_cost,
    )
    from dual_fr3_trunking_mtc.mtc.path_length import anchor_path_length_cost

    if getattr(spec, "primitive", "") == "direct_move_to_next_anchor" and task_plan is not None:
        from geometry_msgs.msg import PoseStamped

        point = task_plan.keypoints[spec.to_index]
        target = PoseStamped()
        target.header.frame_id = point.frame_id
        target.pose.position.x, target.pose.position.y, target.pose.position.z = point.position
        check = anchor_path_length_cost(
            spec.ik_frame, target, args.anchor_max_path_length_ratio, spec.name,
        )
    elif spec.mtc_stage_type == "Merger":
        check = merged_cartesian_path_cost(
            [child.ik_frame for child in spec.children], args.cartesian_path_tolerance, spec.name,
        )
    elif getattr(spec, "planner", "") == "CartesianPath":
        check = cartesian_path_cost(
            spec.ik_frame, args.cartesian_path_tolerance, spec.name,
            stationary=spec.primitive == "turn_gripper_to_next_keypoint",
        )
    else:
        return
    check(solution)


def _stage(container, name):
    try:
        return container[name]
    except (IndexError, KeyError, TypeError):
        return None


def _context(spec, task_plan, recovery_pose_goals):
    text = (
        f"phase={getattr(spec, 'phase', '?')} "
        f"actor={getattr(spec, 'actor', '?')} group={getattr(spec, 'group', '?')} "
        f"planner={getattr(spec, 'planner', '?')} "
        f"{getattr(spec, 'from_keypoint', '?')} -> {getattr(spec, 'to_keypoint', '?')}"
    )
    target = recovery_pose_goals.get(spec.name)
    if target is not None:
        point = target.pose.position
        return text + (
            f" recovery_target=({point.x:.4f}, {point.y:.4f}, {point.z:.4f}) m"
            f" frame={target.header.frame_id}"
        )
    primitive = getattr(spec, "primitive", "")
    if primitive in {
        "direct_move_to_next_anchor", "direct_move_to_seat_edge_keypoint",
        "turn_gripper_to_next_keypoint", "move_above_initial_keypoint",
    } and task_plan is not None:
        index = spec.to_index if primitive.startswith("direct_move") else spec.from_index
        point = task_plan.keypoints[index]
        position = point.position
        if primitive == "move_above_initial_keypoint":
            position = tuple(a + b for a, b in zip(position, spec.vector))
        text += (
            f" target=({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}) m"
            f" frame={point.frame_id} yaw={spec.target_yaw:.4f} rad"
        )
    elif getattr(spec, "mtc_stage_type", "") == "MoveRelative":
        text += (
            f" displacement={getattr(spec, 'vector', '?')} m"
            f" frame={getattr(spec, 'frame_id', '?')}"
        )
    return text


def _invalid_trajectory_details(solution, spec, args):
    """Inspect stored samples only; never infer collision freedom from this scan.

    Native planners do not always retain the offending path. A missing trajectory
    remains an unknown cause. Large paths are sampled to bound diagnostic work.
    """
    trajectory = getattr(solution, "trajectory", None)
    start = getattr(solution, "start", None)
    if trajectory is None or len(trajectory) == 0 or start is None:
        return ["No stored trajectory/scene; see MoveIt collision/constraint messages above."]
    from moveit.core.collision_detection import CollisionRequest, CollisionResult

    scene = start.scene
    request = CollisionRequest()
    request.joint_model_group_name = getattr(spec, "group", "")
    request.contacts = True
    request.max_contacts = 8
    request.max_contacts_per_pair = 1
    request.verbose = False
    indices = np.unique(np.linspace(
        0, len(trajectory) - 1, min(len(trajectory), 4096), dtype=int,
    ))
    collision = None
    max_z = None
    frame_from_model = None
    if getattr(spec, "primitive", "") == "direct_move_to_next_anchor":
        if scene.knows_frame_transform(spec.frame_id):
            frame_from_model = np.linalg.inv(scene.get_frame_transform(spec.frame_id))
    for index in indices:
        state = copy.copy(trajectory[int(index)])
        state.update()
        if collision is None:
            result = CollisionResult()
            scene.check_collision(request, result, state)
            if result.collision:
                try:
                    pairs = ", ".join(f"{a} <-> {b}" for a, b in result.contacts)
                except (TypeError, RuntimeError):
                    # Some installed bindings expose ContactMap without binding
                    # its Contact values. Ask FCL to print the pair once instead.
                    scene.is_state_colliding(state, request.joint_model_group_name, True)
                    pairs = (
                        "contact names unavailable in Python; "
                        "see adjacent native collision message"
                    )
                collision = (
                    f"COLLISION at stored waypoint {index}/{len(trajectory)-1}: "
                    f"{pairs or 'contact names unavailable'}"
                )
        if frame_from_model is not None:
            pose = frame_from_model @ state.get_global_link_transform(spec.ik_frame)
            z = float(pose[2, 3])
            if max_z is None or z > max_z[0]:
                max_z = (z, int(index))
        elif collision is not None:
            break
    details = [collision] if collision else []
    if max_z is not None and args is not None and max_z[0] > args.anchor_max_path_z + 1e-6:
        details.append(
            f"HEIGHT_LIMIT: {spec.ik_frame} z={max_z[0]:.6f} m > "
            f"anchor_max_path_z={args.anchor_max_path_z:.6f} m in {spec.frame_id} "
            f"at stored waypoint {max_z[1]}"
        )
    if not details:
        details.append(
            "Cause not localized in stored samples; "
            "see MoveIt collision/constraint messages above."
        )
    if len(indices) < len(trajectory):
        details.append(f"Diagnostics sampled {len(indices)}/{len(trajectory)} stored waypoints.")
    return details


@dataclass
class PlanningFailureHistory:
    """Count attempts with rejected stage candidates, without retaining native tasks."""

    attempts: int = 0
    reasons: Counter = field(default_factory=Counter)
    stage_outcomes: dict[tuple[int, str], Counter] = field(default_factory=dict)

    def record_stage(self, spec, status):
        """Record one outcome per stage per unsuccessful full-task attempt."""
        key = (getattr(spec, "stage_index", -1), spec.name)
        self.stage_outcomes.setdefault(key, Counter())[status] += 1

    def log_summary(self, logger):
        """Print counts without conflating rejected branches with task execution."""
        if not self.attempts:
            return
        logger.error("[planning-summary] %d unsuccessful full-task attempts", self.attempts)
        logger.info(
            "  Per-stage attempt counts: with_result=has_solution+failed_only; "
            "has_solution means a local candidate, not complete-task success."
        )
        logger.info(
            "  no_result means no candidate was returned (upstream blockage or timeout "
            "may be responsible); skipped/unavailable are excluded from with_result."
        )
        # Preserve task order, including Merger children directly below their parent.
        for (index, name), counts in self.stage_outcomes.items():
            success = counts["HAS_SOLUTION"]
            failed = counts["FAILED"]
            with_result = success + failed
            rate = f"{100.0 * failed / with_result:.1f}%" if with_result else "n/a"
            level = logging.ERROR if failed else (
                logging.WARNING if counts["NO_RESULT"] or counts["UNAVAILABLE"] else logging.INFO
            )
            logger.log(
                level,
                "  [%02d] %s | with_result=%d/%d | has_solution=%d | failed_only=%d | "
                "no_result=%d | skipped=%d | unavailable=%d | "
                "failed_only/with_result=%s",
                index, name, with_result, sum(counts.values()), success, failed,
                counts["NO_RESULT"], counts["SKIPPED"], counts["UNAVAILABLE"], rate,
            )
        logger.error(
            "  Rejection reasons (one attempt may contain multiple reasons or "
            "rejected branches alongside a valid candidate):"
        )
        for (name, reason), count in self.reasons.most_common():
            logger.error("  %dx %s: %s", count, name, reason)


def log_planning_failure(
    task, specs, logger, *, task_plan=None, args=None,
    selected_stage_indices=None, recovery_pose_goals=None, history=None,
):
    """Show stage candidates, rejected candidates and stages with no result.

    A stage can have both successes and failures; a success here is a local
    candidate, never proof of a complete task solution or executed motion.
    """
    if history is not None:
        history.attempts += 1
    logger.error("[planning-failure] Stage results for this attempt (planning only):")
    observed = set()

    def visit(container, spec, indent=""):
        label = f"{indent}[{getattr(spec, 'stage_index', -1):02d}] {spec.name}"
        if not spec.executable:
            if history is not None:
                history.record_stage(spec, "SKIPPED")
            logger.info("%s SKIPPED (non-motion stage)", label)
            return
        stage = _stage(container, spec.name)
        if stage is None:
            if history is not None:
                history.record_stage(
                    spec,
                    "SKIPPED" if spec.mtc_stage_type == "GripperOperation" else "UNAVAILABLE",
                )
            state = (
                "SKIPPED (no MTC motion for gripper hold)"
                if spec.mtc_stage_type == "GripperOperation"
                else "UNAVAILABLE (stage not exposed)"
            )
            logger.info("%s %s", label, state)
            return
        successes = len(getattr(stage, "solutions", ()))
        failures = list(stage.failures)
        status = "HAS_SOLUTION" if successes else "FAILED" if failures else "NO_RESULT"
        if history is not None:
            history.record_stage(spec, status)
        level = logging.ERROR if status == "FAILED" else (
            logging.WARNING if failures else logging.INFO
        )
        logger.log(
            level,
            "%s %s (solutions=%d, failures=%d) | %s",
            label, status, successes, len(failures),
            _context(spec, task_plan, recovery_pose_goals or {}),
        )
        if not successes and not failures:
            logger.info(
                "  No candidate returned; upstream stages or timeout may prevent planning. "
                "Not a confirmed failure."
            )
        for failure in failures[:3]:
            try:
                explain_cost_failure(failure, spec, task_plan, args)
            except Exception as exc:
                logger.warning("  Cost diagnostics unavailable: %s", exc)
        grouped = Counter(
            str(failure.comment).strip() or "No reason returned by MTC" for failure in failures
        )
        for reason, count in list(grouped.items())[:3]:
            logger.error("  reason (%dx): %s", count, reason)
        if len(grouped) > 3:
            logger.warning("  %d additional distinct reasons omitted", len(grouped) - 3)
        for reason in grouped:
            # Strip numeric details of custom checks for a useful retry tally.
            observed.add((spec.name, reason.split(":", 1)[0]))
        for failure in failures[:3]:
            if "INVALID_MOTION_PLAN" not in str(failure.comment):
                continue
            try:
                for detail in _invalid_trajectory_details(failure, spec, args):
                    logger.error("  %s", detail)
            except Exception as exc:  # diagnostics must not interrupt planning/retries
                logger.warning(
                    "  Trajectory diagnostics unavailable: %s; see MoveIt messages above", exc,
                )
        for child in getattr(spec, "children", ()):
            visit(stage, child, indent + "  ")

    try:
        start = _stage(task, "current_state")
        if start is not None:
            for failure in start.failures:
                reason = str(failure.comment) or "CurrentState unavailable"
                logger.error("[current_state] FAILED: %s", reason)
                observed.add(("current_state", reason))
        for spec in specs:
            if selected_stage_indices is None or spec.stage_index in selected_stage_indices:
                visit(task, spec)
    except Exception as exc:  # native introspection is best effort
        logger.warning("Stage diagnostics incomplete: %s", exc)
    if history is not None:
        history.reasons.update(observed or {("task", "No stage rejection reason available")})
