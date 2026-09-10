"""Search preparation IK configurations by planning the complete continuation."""

import copy
from dataclasses import dataclass
import math
import time

import numpy as np
from geometry_msgs.msg import Pose

from .diagnostics import PlanningFailureHistory, log_planning_failure
from .planning import PlannedTask, create_task_from_args
from .task_builder import import_mtc_modules, preparation_pose_goal


IK_POSITION_TOLERANCE = 1e-4  # m
IK_ORIENTATION_TOLERANCE = 1e-3  # rad


@dataclass
class CapturedScene:
    scene: object
    model_owner: object


def validate_preparation_search_settings(args):
    for name in (
        "preparation_ik_candidates", "preparation_ik_attempts",
        "preparation_candidate_attempts",
    ):
        if getattr(args, name) < 1:
            raise ValueError(f"{name} must be >= 1")
    for name in (
        "preparation_ik_timeout", "preparation_min_joint_distance",
        "preparation_search_timeout",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and > 0")


def capture_start_scene(node):
    """Capture one live scene, including objects, grippers and the collision matrix."""
    _rclcpp, core, stages = import_mtc_modules()
    task = core.Task()
    task.name = "preparation_start_snapshot"
    task.loadRobotModel(node)
    task.add(stages.CurrentState("current_state"))
    if not task.plan(1) or not task.solutions:
        raise RuntimeError("cannot capture CurrentState for preparation search")
    return CapturedScene(copy.copy(task.solutions[0].start.scene), task)


def pose_in_model(scene, target):
    """Transform a TCP pose from its task frame into the IK model frame."""
    if not scene.knows_frame_transform(target.header.frame_id):
        raise ValueError(f"unknown preparation frame {target.header.frame_id!r}")
    point = target.pose.position
    q = target.pose.orientation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 1e-12:
        raise ValueError("invalid preparation orientation")
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    transform = np.eye(4)
    transform[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    transform[:3, 3] = [point.x, point.y, point.z]
    transform = scene.get_frame_transform(target.header.frame_id) @ transform
    if not np.isfinite(transform).all():
        raise ValueError("non-finite preparation pose")
    r = transform[:3, :3]
    # Symmetric quaternion extraction remains stable at 180-degree tool poses.
    matrix = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)],
    ])
    _values, vectors = np.linalg.eigh(matrix)
    orientation = vectors[:, -1]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, transform[:3, 3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        map(float, orientation)
    )
    return pose


def _pose_matches(actual, target):
    position_error = math.sqrt(sum(
        (getattr(actual.position, axis) - getattr(target.position, axis)) ** 2
        for axis in ("x", "y", "z")
    ))
    a = np.array([getattr(actual.orientation, axis) for axis in ("x", "y", "z", "w")])
    b = np.array([getattr(target.orientation, axis) for axis in ("x", "y", "z", "w")])
    if not np.isfinite(a).all() or np.linalg.norm(a) < 1e-12:
        return False
    dot = abs(float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))))
    angle_error = 2.0 * math.acos(min(1.0, dot))
    return position_error <= IK_POSITION_TOLERANCE and angle_error <= IK_ORIENTATION_TOLERANCE


def sample_preparation_ik(scene, target, spec, args, logger, deadline, *, clock=time.monotonic):
    """
    Sample IK from current joints, then random bounded FR3 joint seeds.

    Candidates are joint configurations for the *same* TCP pose. Collision
    checking is deferred to the combined scene and the complete MTC plan: the
    other arm's initial pose need not be its pose when this candidate is reached.
    """
    model = scene.robot_model
    if not model.has_joint_model_group(spec.group):
        raise ValueError(f"unknown preparation group {spec.group!r}")
    group = model.get_joint_model_group(spec.group)
    names = tuple(group.active_joint_model_names)
    bounds = group.active_joint_model_bounds
    if not names or len(bounds) != len(names) or any(len(item) != 1 for item in bounds):
        raise ValueError("preparation sampling requires single-variable arm joints")
    lower = np.array([item[0].min_position for item in bounds])
    upper = np.array([item[0].max_position for item in bounds])
    if (not all(item[0].position_bounded for item in bounds)
            or not np.isfinite(lower).all() or not np.isfinite(upper).all()
            or np.any(lower >= upper)):
        raise ValueError("preparation sampling requires finite bounded arm joints")
    if not scene.knows_frame_transform(spec.ik_frame):
        raise ValueError(f"unknown preparation TCP {spec.ik_frame!r}")
    goal = pose_in_model(scene, target)
    rng = np.random.default_rng()
    candidates = []
    vectors = []
    attempted = 0
    for attempt in range(args.preparation_ik_attempts):
        remaining = deadline - clock()
        if remaining <= 0.0:
            break
        attempted += 1
        state = copy.copy(scene.current_state)
        if attempt:
            # The named-joint interface avoids Eigen array conversion in the
            # installed Python bindings and leaves the other arm untouched.
            state.joint_positions = {
                name: float(value) for name, value in zip(names, rng.uniform(lower, upper))
            }
        state.update()
        if not state.set_from_ik(
            spec.group, goal, spec.ik_frame, min(args.preparation_ik_timeout, remaining),
        ):
            continue
        state.update()
        joints = state.joint_positions
        values = np.array([joints[name] for name in names])
        # Check active joint bounds without passing arrays through native Eigen casters.
        if not np.isfinite(values).all() or np.any(values < lower) or np.any(values > upper):
            continue
        if not _pose_matches(state.get_pose(spec.ik_frame), goal):
            continue
        if any(np.linalg.norm(values - previous) < args.preparation_min_joint_distance
               for previous in vectors):
            continue
        vectors.append(values)
        candidates.append({name: float(value) for name, value in zip(names, values)})
        logger.info(
            "%s: preparation IK candidate %d/%d from seed %d: %s",
            spec.actor, len(candidates), args.preparation_ik_candidates, attempted,
            ", ".join(f"{name}={value:.4f}" for name, value in candidates[-1].items()),
        )
        if len(candidates) >= args.preparation_ik_candidates:
            break
    logger.info(
        "%s: %d distinct IK candidates from %d seeds", spec.actor, len(candidates), attempted,
    )
    return candidates


def candidate_pair_indices(first_count, second_count):
    """Visit both arms' alternatives early using diagonal Cartesian-product order."""
    for diagonal in range(first_count + second_count - 1):
        for first in range(first_count):
            second = diagonal - first
            if 0 <= second < second_count:
                yield first, second


def _retryable_pipeline_failure(task, specs):
    """Retry stochastic pipeline failures before discarding a joint candidate."""
    failed = [
        spec for spec in specs
        if spec.executable and spec.mtc_stage_type != "GripperOperation"
        and task[spec.name].failures
    ]
    return bool(failed) and all(
        getattr(spec, "planner", "") == "PipelinePlanner" for spec in failed
    )


def plan_preparation_candidates(
    node, task_plan, specs, args, logger, gripper_profiles=None, *,
    scene_provider=capture_start_scene, sampler=sample_preparation_ik,
    task_factory=create_task_from_args, clock=time.monotonic,
):
    """
    Return a complete preparation+formal solution before permitting any motion.

    Search is bounded by IK seeds, candidates per arm, rounds per candidate pair,
    pipeline planning retries and a wall-clock budget checked between native
    solver calls. A native call may overrun the budget by its current computation.
    """
    validate_preparation_search_settings(args)
    if args.planning_attempts < 1:
        raise ValueError("planning_attempts must be >= 1")
    specs = tuple(specs)
    approaches = [
        s for s in specs if s.executable and s.primitive == "move_above_initial_keypoint"
    ]
    if len(approaches) != 2 or {s.actor for s in approaches} != {"leader", "follower"}:
        raise ValueError("preparation search requires one approach stage for each arm")
    approaches.sort(key=lambda s: s.actor != "leader")
    deadline = clock() + args.preparation_search_timeout
    history = PlanningFailureHistory()
    try:
        snapshot = scene_provider(node)
        scene = snapshot.scene
        candidates = []
        for spec in approaches:
            target = preparation_pose_goal(task_plan, spec, args.tool_roll, args.tool_pitch)
            candidates.append(sampler(scene, target, spec, args, logger, deadline, clock=clock))
        if any(not group for group in candidates):
            for spec, group in zip(approaches, candidates):
                if not group:
                    target = preparation_pose_goal(task_plan, spec, args.tool_roll, args.tool_pitch)
                    point = target.pose.position
                    logger.error(
                        "[preparation-ik] %s group=%s TCP=%s: no IK candidate for "
                        "target=(%.4f, %.4f, %.4f) m frame=%s",
                        spec.name, spec.group, spec.ik_frame,
                        point.x, point.y, point.z, target.header.frame_id,
                    )
            logger.error(
                "preparation search found no IK candidates for at least one arm; no motion issued",
            )
            return None
        pairs = list(candidate_pair_indices(len(candidates[0]), len(candidates[1])))
        logger.info(
            "checking %d preparation pairs, up to %d rounds; "
            "full continuation required before execution",
            len(pairs), args.preparation_candidate_attempts,
        )
        for round_index in range(args.preparation_candidate_attempts):
            for first, second in pairs:
                remaining = deadline - clock()
                if remaining <= 0.0:
                    logger.error("preparation search time budget exhausted; no motion issued")
                    history.log_summary(logger)
                    return None
                goals = {
                    approaches[0].name: candidates[0][first],
                    approaches[1].name: candidates[1][second],
                }
                combined = copy.copy(scene.current_state)
                for values in goals.values():
                    combined.joint_positions = values
                combined.update()
                if not scene.is_state_valid(combined, ""):
                    logger.info(
                        "preparation pair L%d/F%d rejected: combined state invalid",
                        first+1, second+1,
                    )
                    continue
                for attempt in range(1, args.planning_attempts + 1):
                    if deadline <= clock():
                        logger.error("preparation search time budget exhausted; no motion issued")
                        history.log_summary(logger)
                        return None
                    logger.info(
                        "preparation pair L%d/F%d, round %d/%d, planning attempt %d/%d: "
                        "planning preparation and all formal stages",
                        first+1, second+1, round_index+1, args.preparation_candidate_attempts,
                        attempt, args.planning_attempts,
                    )
                    task, _ = task_factory(
                        node, task_plan, specs, args, gripper_profiles=gripper_profiles,
                        preparation_joint_goals=goals, start_scene=scene,
                    )
                    task.properties["timeout"] = max(0.001, deadline - clock())
                    if task.plan(1) and task.solutions:
                        logger.info(
                            "selected preparation pair L%d/F%d: "
                            "complete preparation and formal path accepted; "
                            "retaining exact joint goals and trajectories",
                            first+1, second+1,
                        )
                        return PlannedTask(
                            task, task.solutions[0], specs, copy.deepcopy(goals),
                            snapshot.model_owner,
                        )
                    logger.warning(
                        "preparation pair L%d/F%d rejected (round %d, attempt %d)",
                        first+1, second+1, round_index+1, attempt,
                    )
                    log_planning_failure(
                        task, specs, logger, task_plan=task_plan, args=args, history=history,
                    )
                    if not _retryable_pipeline_failure(task, specs):
                        break
                    if attempt < args.planning_attempts:
                        logger.warning(
                            "pipeline planning failed; retrying this preparation pair "
                            "before discarding its joint configuration",
                        )
        logger.error("all preparation candidates exhausted; no motion issued")
        history.log_summary(logger)
    except Exception:  # noqa: BLE001 - configuration/binding failures must not start execution
        logger.exception("preparation candidate search failed; no motion issued")
    return None
