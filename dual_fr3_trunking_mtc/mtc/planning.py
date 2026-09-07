"""Bounded planning attempts shared by preparation, preview and recovery."""

from dataclasses import dataclass, field

from ..runtime.config import DEFAULTS
from .task_builder import create_mtc_task


@dataclass
class PlannedTask:
    task: object
    solution: object
    specs: tuple
    preparation_joint_goals: dict = field(default_factory=dict)
    # A FixedState task reuses the captured model. Its original loader must
    # outlive planning/execution so native kinematics allocators remain valid.
    model_owner: object | None = None


def create_task_from_args(
    node, task_plan, specs, args, gripper_profiles=None,
    selected_stage_indices=None, recovery_pose_goals=None,
    task_factory=create_mtc_task,
    preparation_joint_goals=None, start_scene=None,
):
    return task_factory(
        node, task_plan, specs,
        leader_group=args.leader_group,
        follower_group=args.follower_group,
        leader_ik_frame=args.leader_ik_frame,
        follower_ik_frame=args.follower_ik_frame,
        cartesian_step_size=args.cartesian_step_size,
        cartesian_jump_threshold=getattr(
            args, "cartesian_jump_threshold", DEFAULTS.cartesian_jump_threshold,
        ),
        cartesian_path_tolerance=getattr(
            args, "cartesian_path_tolerance", DEFAULTS.cartesian_path_tolerance,
        ),
        motion_velocity_scaling=args.motion_velocity_scaling,
        motion_acceleration_scaling=args.motion_acceleration_scaling,
        leader_lead_distance=args.leader_lead_distance,
        tool_roll=args.tool_roll,
        tool_pitch=args.tool_pitch,
        selected_stage_indices=selected_stage_indices,
        leader_orientation_direction=args.leader_orientation_direction,
        follower_orientation_direction=args.follower_orientation_direction,
        anchor_max_path_z=args.anchor_max_path_z,
        anchor_max_path_length_ratio=args.anchor_max_path_length_ratio,
        gripper_profiles=gripper_profiles,
        recovery_pose_goals=recovery_pose_goals,
        preparation_joint_goals=preparation_joint_goals,
        start_scene=start_scene,
    )


def plan_with_retries(
    node, task_plan, specs, args, logger, gripper_profiles=None,
    selected_stage_indices=None, recovery_pose_goals=None,
    task_factory=create_mtc_task,
    preparation_joint_goals=None,
):
    attempts = getattr(args, "planning_attempts", DEFAULTS.planning_attempts)
    if attempts < 1:
        raise ValueError("planning_attempts must be >= 1")
    specs = tuple(specs)
    for attempt in range(1, attempts + 1):
        logger.info("planning attempt %d/%d", attempt, attempts)
        try:
            # Construct a fresh CurrentState on each attempt. A failed plan may
            # contain consumed MTC interfaces and must not supply the next start.
            task, _ = create_task_from_args(
                node, task_plan, specs, args, gripper_profiles,
                selected_stage_indices, recovery_pose_goals, task_factory,
                preparation_joint_goals=preparation_joint_goals,
            )
            succeeded = task.plan()
        except Exception:  # noqa: BLE001 - configuration/binding errors are not retryable
            logger.exception("planning raised; stopping planning attempts")
            return None
        if succeeded and task.solutions:
            solution = task.solutions[0]
            logger.info(
                "planning succeeded on attempt %d/%d; retaining this solution",
                attempt, attempts,
            )
            return PlannedTask(task, solution, specs, preparation_joint_goals or {})
        logger.warning("planning attempt %d/%d produced no valid solution", attempt, attempts)
    logger.error("planning failed after %d attempts", attempts)
    return None
