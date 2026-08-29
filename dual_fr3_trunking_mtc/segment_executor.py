from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path
from typing import Iterable, Sequence

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import PlanningOptions

from .models import (
    DEFAULT_LEADER_LEAD_DISTANCE,
    DEFAULT_TOOL_PITCH,
    DEFAULT_TOOL_ROLL,
    Keypoint,
)
from .mtc_prototype import MtcStageSpec, build_mtc_stage_specs
from .planner import load_keypoints
from .scheduler import build_task_schedule


ARM_FOR_ACTOR = {"leader": "left", "follower": "right"}
MOVEIT_ERROR_NAMES = {
    1: "SUCCESS",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -12: "GOAL_IN_COLLISION",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -21: "FRAME_TRANSFORM_FAILURE",
    -23: "ROBOT_STATE_STALE",
    -31: "NO_IK_SOLUTION",
}
ARM_PRIMITIVES = {
    "move_to_initial_keypoint",
    "turn_gripper_to_next_keypoint",
    "leader_move_ahead_for_seat_edge",
    "direct_move_to_seat_edge_keypoint",
    "cartesian_move_to_next_keypoint",
    "direct_move_to_next_anchor",
}
DIRECT_PRIMITIVES = {
    "move_to_initial_keypoint",
    "direct_move_to_seat_edge_keypoint",
    "direct_move_to_next_anchor",
}
RELATIVE_PRIMITIVES = {
    "leader_move_ahead_for_seat_edge",
    "cartesian_move_to_next_keypoint",
}


def _vector_yaw(vector: tuple[float, float, float]) -> float:
    return math.atan2(vector[1], vector[0])


def _stage_motion_type(spec: MtcStageSpec) -> str:
    if spec.primitive == "close_gripper_at_start":
        return "gripper"
    if spec.primitive == "seat_cable_on_edge":
        return "skip"
    if not spec.executable:
        return "skip"
    if spec.primitive in DIRECT_PRIMITIVES:
        return "ompl"
    if spec.primitive in ARM_PRIMITIVES:
        return "cartesian"
    return "unsupported"


def stage_description(spec: MtcStageSpec) -> str:
    motion_type = _stage_motion_type(spec)
    target = spec.to_keypoint
    if spec.primitive == "leader_move_ahead_for_seat_edge":
        target = (
            f"relative [{spec.vector[0]:+.3f}, {spec.vector[1]:+.3f}, "
            f"{spec.vector[2]:+.3f}] m"
        )
    elif spec.primitive == "turn_gripper_to_next_keypoint":
        target = f"rotate at fixed xyz to yaw={spec.target_yaw:+.3f} rad"
    return (
        f"[{spec.stage_index:02d}] {motion_type:9s} "
        f"{spec.actor:8s} {spec.from_keypoint} -> {target} "
        f"({spec.primitive})"
    )


def build_segment_stages(
    keypoints: Sequence[Keypoint],
    initial_leader_index: int = 1,
    initial_follower_index: int = 0,
    leader_lead_distance: float = DEFAULT_LEADER_LEAD_DISTANCE,
) -> list[MtcStageSpec]:
    task_steps = build_task_schedule(
        keypoints,
        initial_leader_index=initial_leader_index,
        initial_follower_index=initial_follower_index,
    )
    return build_mtc_stage_specs(
        keypoints,
        task_steps,
        initial_leader_index=initial_leader_index,
        initial_follower_index=initial_follower_index,
        leader_lead_distance=leader_lead_distance,
    )


def _load_controller_class():
    package_prefix = Path(get_package_prefix("dual_fr3_moveit_config"))
    script_directory = package_prefix / "lib" / "dual_fr3_moveit_config"
    if not (script_directory / "fr3_controller_lin.py").exists():
        raise RuntimeError(
            "dual_fr3_moveit_config controller scripts are not installed; "
            "build that package and source install/setup.bash"
        )
    sys.path.insert(0, str(script_directory))
    from fr3_controller_lin import DualFR3LinearController

    return DualFR3LinearController


def wait_for_controller_state(controller, timeout: float = 3.0) -> bool:
    state_received = threading.Event()
    subscription = controller.create_subscription(
        JointTrajectoryControllerState,
        "/right_fr3_arm_controller/controller_state",
        lambda _message: state_received.set(),
        10,
    )
    try:
        return state_received.wait(timeout)
    finally:
        controller.destroy_subscription(subscription)


class StageRunner:
    def __init__(
        self,
        controller,
        keypoints: Sequence[Keypoint],
        tool_roll: float,
        tool_pitch: float,
        execute: bool,
        skip_gripper: bool,
        settle_time: float,
        position_tolerance: float,
        orientation_tolerance: float,
        verification_timeout: float,
        cartesian_max_step: float,
        cartesian_min_fraction: float,
        allow_cartesian_fallback: bool,
        ompl_planner_id: str,
        planning_time: float,
        velocity_scale: float,
        acceleration_scale: float,
    ) -> None:
        self.controller = controller
        self.keypoints = keypoints
        self.tool_roll = tool_roll
        self.tool_pitch = tool_pitch
        self.execute = execute
        self.skip_gripper = skip_gripper
        self.settle_time = settle_time
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.verification_timeout = verification_timeout
        self.cartesian_max_step = cartesian_max_step
        self.cartesian_min_fraction = cartesian_min_fraction
        self.allow_cartesian_fallback = allow_cartesian_fallback
        self.ompl_planner_id = ompl_planner_id
        self.planning_time = planning_time
        self.velocity_scale = velocity_scale
        self.acceleration_scale = acceleration_scale

    @staticmethod
    def _error_name(error_code: int) -> str:
        return MOVEIT_ERROR_NAMES.get(error_code, "UNKNOWN")

    def _send_action(self, client, goal, label: str):
        import rclpy

        if not client.wait_for_server(timeout_sec=5.0):
            self.controller.get_logger().error(f"{label} action server unavailable")
            return None
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.controller, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.controller.get_logger().error(f"{label} goal was rejected")
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.controller, result_future)
        return result_future.result()

    def _move_to_ompl(
        self,
        arm: str,
        position,
        yaw: float,
        frame_id: str,
    ) -> bool:
        controller = self.controller
        arm_config = controller._arm(arm)
        target_pose = controller._create_pose_target(
            position[0],
            position[1],
            position[2],
            self.tool_roll,
            self.tool_pitch,
            yaw,
        )
        constraints = controller._create_pose_constraints(
            arm,
            target_pose,
            frame_id,
        )
        request = controller._create_motion_plan_request(
            arm_config.planning_group,
            [constraints],
            frame_id=frame_id,
            planner_id=self.ompl_planner_id,
            velocity_scale=self.velocity_scale,
            acceleration_scale=self.acceleration_scale,
            planning_time=self.planning_time,
        )
        controller.get_logger().info(
            f"Planning {arm} arm with {self.ompl_planner_id} for up to "
            f"{self.planning_time:.1f} s"
        )
        plan_goal = MoveGroup.Goal()
        plan_goal.request = request
        plan_goal.planning_options = PlanningOptions()
        plan_goal.planning_options.plan_only = True
        plan_goal.planning_options.look_around = False
        plan_goal.planning_options.replan = False
        plan_result = self._send_action(
            controller.move_group_client,
            plan_goal,
            "MoveGroup planning",
        )
        if plan_result is None:
            return False
        plan_error = plan_result.result.error_code.val
        controller.get_logger().info(
            f"planning result: {self._error_name(plan_error)} ({plan_error})"
        )
        if plan_error != 1:
            return False
        if not self.execute:
            return True

        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = plan_result.result.planned_trajectory
        execute_result = self._send_action(
            controller.execute_trajectory_client,
            execute_goal,
            "ExecuteTrajectory",
        )
        if execute_result is None:
            return False
        execute_error = execute_result.result.error_code.val
        controller.get_logger().info(
            f"execution result: {self._error_name(execute_error)} "
            f"({execute_error})"
        )
        return execute_error == 1

    def _wait_for_pose(self, arm: str, timeout: float = 5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pose = self.controller.get_current_pose(arm)
            if pose is not None:
                return pose
            time.sleep(0.05)
        return None

    def _log_pose(self, label: str, arm: str) -> None:
        values = self.controller.get_current_pose_rpy(arm)
        if values is None:
            self.controller.get_logger().warn(f"{label}: {arm} TCP pose unavailable")
            return
        self.controller.get_logger().info(
            f"{label}: {arm} TCP xyz=[{values[0]:.4f}, {values[1]:.4f}, "
            f"{values[2]:.4f}], rpy=[{values[3]:.3f}, {values[4]:.3f}, "
            f"{values[5]:.3f}]"
        )

    def _target(self, spec: MtcStageSpec, current_pose):
        if spec.primitive == "turn_gripper_to_next_keypoint":
            position = (
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z,
            )
            yaw = spec.target_yaw
        elif spec.primitive in RELATIVE_PRIMITIVES:
            position = (
                current_pose.position.x + spec.vector[0],
                current_pose.position.y + spec.vector[1],
                current_pose.position.z + spec.vector[2],
            )
            yaw = _vector_yaw(spec.vector)
        else:
            position = self.keypoints[spec.to_index].position
            yaw = spec.target_yaw
        return position, yaw

    @staticmethod
    def _quaternion_distance(actual, target) -> float:
        dot = abs(
            actual.x * target.x
            + actual.y * target.y
            + actual.z * target.z
            + actual.w * target.w
        )
        return 2.0 * math.acos(min(1.0, max(-1.0, dot)))

    def _verify_pose(self, arm: str, position, yaw: float) -> bool:
        target = self.controller._create_pose_target(
            position[0],
            position[1],
            position[2],
            self.tool_roll,
            self.tool_pitch,
            yaw,
        )
        deadline = time.monotonic() + self.verification_timeout
        position_error = math.inf
        orientation_error = math.inf
        while time.monotonic() < deadline:
            actual = self.controller.get_current_pose(arm)
            if actual is not None:
                position_error = math.sqrt(
                    (actual.position.x - position[0]) ** 2
                    + (actual.position.y - position[1]) ** 2
                    + (actual.position.z - position[2]) ** 2
                )
                orientation_error = self._quaternion_distance(
                    actual.orientation,
                    target.orientation,
                )
                if (
                    position_error <= self.position_tolerance
                    and orientation_error <= self.orientation_tolerance
                ):
                    break
            time.sleep(0.05)

        self.controller.get_logger().info(
            f"verification: position_error={position_error:.4f} m, "
            f"orientation_error={orientation_error:.4f} rad"
        )
        return (
            position_error <= self.position_tolerance
            and orientation_error <= self.orientation_tolerance
        )

    def run(self, spec: MtcStageSpec) -> bool:
        logger = self.controller.get_logger()
        logger.info("=" * 72)
        logger.info(stage_description(spec))

        motion_type = _stage_motion_type(spec)
        if motion_type == "skip":
            if spec.primitive == "turn_gripper_to_next_keypoint":
                logger.info("SKIP: gripper already has the required yaw")
            else:
                logger.info("SKIP: informational cable-seating placeholder")
            return True
        if motion_type == "unsupported":
            logger.error(f"Unsupported primitive: {spec.primitive}")
            return False
        if motion_type == "gripper":
            if self.skip_gripper:
                logger.info("SKIP: --skip-gripper is enabled")
                return True
            arm = ARM_FOR_ACTOR[spec.actor]
            ok = self.controller.close_gripper(arm) if self.execute else True
            logger.info(f"RESULT: {'SUCCESS' if ok else 'FAILED'}")
            return ok

        arm = ARM_FOR_ACTOR[spec.actor]
        current_pose = self._wait_for_pose(arm)
        if current_pose is None:
            logger.error(f"No TF pose available for {arm} arm")
            return False

        self._log_pose("before", arm)
        position, yaw = self._target(spec, current_pose)
        logger.info(
            f"command: method={motion_type}, arm={arm}, "
            f"frame={spec.frame_id}, xyz=[{position[0]:.4f}, "
            f"{position[1]:.4f}, {position[2]:.4f}], "
            f"rpy=[{self.tool_roll:.3f}, {self.tool_pitch:.3f}, {yaw:.3f}]"
        )

        if motion_type == "ompl":
            ok = self._move_to_ompl(
                arm,
                position,
                yaw,
                spec.frame_id,
            )
        else:
            original_move_to_ompl = self.controller.move_to_ompl
            fallback_used = False

            def cartesian_fallback(*fallback_args, **fallback_kwargs):
                nonlocal fallback_used
                fallback_used = True
                if not self.allow_cartesian_fallback:
                    logger.error(
                        "Cartesian planning requested an OMPL fallback; "
                        "strict diagnostic mode rejects it"
                    )
                    return False
                fallback_position = fallback_args[1:4]
                fallback_yaw = fallback_args[6]
                fallback_frame = fallback_kwargs.get("frame_id", spec.frame_id)
                return self._move_to_ompl(
                    fallback_args[0],
                    fallback_position,
                    fallback_yaw,
                    fallback_frame,
                )

            self.controller.move_to_ompl = cartesian_fallback
            try:
                ok = self.controller.move_to_cartesian(
                    arm,
                    position[0],
                    position[1],
                    position[2],
                    self.tool_roll,
                    self.tool_pitch,
                    yaw,
                    execute=self.execute,
                    frame_id=spec.frame_id,
                    max_step=self.cartesian_max_step,
                    min_fraction=self.cartesian_min_fraction,
                )
            finally:
                self.controller.move_to_ompl = original_move_to_ompl
            if fallback_used:
                logger.warn(
                    "Cartesian stage used the controller script's OMPL fallback"
                )

        if self.execute:
            time.sleep(self.settle_time)
            self._log_pose("after", arm)
            verified = self._verify_pose(arm, position, yaw)
            ok = ok and verified
        logger.info(f"RESULT: {'SUCCESS' if ok else 'FAILED'}")
        return ok


def _parse_args(argv: Iterable[str]) -> argparse.Namespace:
    try:
        package_directory = Path(
            get_package_share_directory("dual_fr3_trunking_mtc")
        )
    except PackageNotFoundError:
        package_directory = Path(__file__).resolve().parents[1]
    default_keypoints = package_directory / "config" / "keypoints.yaml"
    parser = argparse.ArgumentParser(
        description=(
            "Execute the trunking task one MoveGroup/Cartesian stage at a time "
            "using dual_fr3_moveit_config's existing Python controllers."
        )
    )
    parser.add_argument("--keypoints-file", default=str(default_keypoints))
    parser.add_argument("--task-frame", default="left_fr3_link0")
    parser.add_argument("--initial-leader-index", type=int, default=1)
    parser.add_argument("--initial-follower-index", type=int, default=0)
    parser.add_argument(
        "--leader-lead-distance",
        type=float,
        default=DEFAULT_LEADER_LEAD_DISTANCE,
    )
    parser.add_argument("--tool-roll", type=float, default=DEFAULT_TOOL_ROLL)
    parser.add_argument("--tool-pitch", type=float, default=DEFAULT_TOOL_PITCH)
    parser.add_argument("--start-stage", type=int, default=0)
    parser.add_argument("--stop-stage", type=int)
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--confirm-each", action="store_true")
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument("--skip-gripper", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--position-tolerance", type=float, default=0.025)
    parser.add_argument("--orientation-tolerance", type=float, default=0.12)
    parser.add_argument("--verification-timeout", type=float, default=2.0)
    parser.add_argument("--cartesian-max-step", type=float, default=0.005)
    parser.add_argument("--cartesian-min-fraction", type=float, default=0.9)
    parser.add_argument("--allow-cartesian-fallback", action="store_true")
    parser.add_argument(
        "--ompl-planner-id",
        default="RRTConnectkConfigDefault",
    )
    parser.add_argument("--planning-time", type=float, default=20.0)
    parser.add_argument("--velocity-scale", type=float, default=0.1)
    parser.add_argument("--acceleration-scale", type=float, default=0.1)
    parser.add_argument("--allow-no-controller-state", action="store_true")
    return parser.parse_args(list(argv))


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    keypoints = load_keypoints(args.keypoints_file, fallback_frame=args.task_frame)
    specs = build_segment_stages(
        keypoints,
        initial_leader_index=args.initial_leader_index,
        initial_follower_index=args.initial_follower_index,
        leader_lead_distance=args.leader_lead_distance,
    )
    selected = [
        spec
        for spec in specs
        if spec.stage_index >= args.start_stage
        and (args.stop_stage is None or spec.stage_index <= args.stop_stage)
    ]

    print("Trunking segment sequence:")
    for spec in specs:
        marker = "*" if spec in selected else " "
        print(f"{marker} {stage_description(spec)}")
    if args.list:
        return 0
    if not selected:
        print("No stages selected", file=sys.stderr)
        return 2
    if args.plan_only and len(selected) > 1:
        print(
            "Warning: plan-only does not advance robot state; later stage plans "
            "do not start from preceding planned targets.",
            file=sys.stderr,
        )

    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    controller_class = _load_controller_class()
    rclpy.init()
    controller = None
    executor = MultiThreadedExecutor(num_threads=4)
    spin_thread = None
    failures: list[int] = []
    try:
        controller = controller_class(
            node_name="dual_fr3_trunking_segment_executor",
            default_frame=args.task_frame,
        )
        executor.add_node(controller)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        if (
            not args.plan_only
            and not args.allow_no_controller_state
            and not wait_for_controller_state(controller)
        ):
            controller.get_logger().error(
                "Gazebo arm controller state is not updating, so trajectories "
                "cannot execute. Restart gazebo.launch.py. Use "
                "--allow-no-controller-state only for real hardware."
            )
            return 3
        runner = StageRunner(
            controller,
            keypoints,
            tool_roll=args.tool_roll,
            tool_pitch=args.tool_pitch,
            execute=not args.plan_only,
            skip_gripper=args.skip_gripper,
            settle_time=args.settle_time,
            position_tolerance=args.position_tolerance,
            orientation_tolerance=args.orientation_tolerance,
            verification_timeout=args.verification_timeout,
            cartesian_max_step=args.cartesian_max_step,
            cartesian_min_fraction=args.cartesian_min_fraction,
            allow_cartesian_fallback=args.allow_cartesian_fallback,
            ompl_planner_id=args.ompl_planner_id,
            planning_time=args.planning_time,
            velocity_scale=args.velocity_scale,
            acceleration_scale=args.acceleration_scale,
        )

        for spec in selected:
            if args.confirm_each:
                answer = input(f"Run stage {spec.stage_index}? [Enter/q] ").strip().lower()
                if answer in {"q", "quit", "n", "no"}:
                    break
            if not runner.run(spec):
                failures.append(spec.stage_index)
                if not args.continue_on_failure:
                    break
    finally:
        executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if controller is not None:
            controller.destroy_node()
        rclpy.shutdown()

    if failures:
        print(f"Failed stages: {failures}", file=sys.stderr)
        return 1
    print("Selected stages completed successfully")
    return 0
