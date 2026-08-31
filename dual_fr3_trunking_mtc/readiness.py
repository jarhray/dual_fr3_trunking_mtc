from __future__ import annotations

import importlib
import time
from typing import Dict, Iterable

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTrajectoryControllerState
from franka_msgs.action import Homing
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_SIDES = ("left", "right")


def expected_arm_joint_names() -> set[str]:
    return {
        f"{side}_fr3_joint{joint_index}"
        for side in ARM_SIDES
        for joint_index in range(1, 8)
    }


def missing_arm_joint_names(joint_names: Iterable[str]) -> set[str]:
    return expected_arm_joint_names().difference(joint_names)


class TrunkingReadinessGate(Node):
    """Block MTC startup until the complete execution chain is ready."""

    def __init__(self) -> None:
        super().__init__("dual_fr3_trunking_readiness_gate")

        self.declare_parameter("execute", False)
        self.declare_parameter("use_fake_hardware", True)
        self.declare_parameter("namespaced_arm_controllers", True)
        self.declare_parameter("start_gripper", True)
        self.declare_parameter("home_grippers_before_execute", True)
        self.declare_parameter("grippers_homed", False)
        self.declare_parameter("readiness_timeout", 60.0)
        self.declare_parameter("state_max_age", 0.5)

        self.execute = bool(self.get_parameter("execute").value)
        self.use_fake_hardware = bool(
            self.get_parameter("use_fake_hardware").value
        )
        self.namespaced_arm_controllers = bool(
            self.get_parameter("namespaced_arm_controllers").value
        )
        self.start_gripper = bool(self.get_parameter("start_gripper").value)
        self.home_grippers_before_execute = bool(
            self.get_parameter("home_grippers_before_execute").value
        )
        self.grippers_homed = bool(self.get_parameter("grippers_homed").value)
        self.readiness_timeout = float(
            self.get_parameter("readiness_timeout").value
        )
        self.state_max_age = float(self.get_parameter("state_max_age").value)

        self._joint_names: set[str] = set()
        self._joint_state_received_at = 0.0
        self._controller_state_received_at: Dict[str, float] = {
            side: 0.0 for side in ARM_SIDES
        }

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )
        for side in ARM_SIDES:
            controller_prefix = (
                f"/{side}/{side}_fr3_arm_controller"
                if self.namespaced_arm_controllers
                else f"/{side}_fr3_arm_controller"
            )
            self.create_subscription(
                JointTrajectoryControllerState,
                f"{controller_prefix}/controller_state",
                lambda _message, arm_side=side: (
                    self._controller_state_callback(arm_side)
                ),
                10,
            )

        self._required_actions: list[tuple[str, ActionClient]] = [
            (
                "/move_action",
                ActionClient(self, MoveGroup, "/move_action"),
            )
        ]
        for side in ARM_SIDES:
            controller_prefix = (
                f"/{side}/{side}_fr3_arm_controller"
                if self.namespaced_arm_controllers
                else f"/{side}_fr3_arm_controller"
            )
            action_name = f"{controller_prefix}/follow_joint_trajectory"
            self._required_actions.append(
                (
                    action_name,
                    ActionClient(self, FollowJointTrajectory, action_name),
                )
            )

        self._gripper_command_clients: Dict[str, ActionClient] = {}
        self._gripper_homing_clients: Dict[str, ActionClient] = {}
        if self.start_gripper:
            for side in ARM_SIDES:
                command_name = f"/{side}_franka_gripper/gripper_action"
                command_client = ActionClient(
                    self,
                    GripperCommand,
                    command_name,
                )
                self._gripper_command_clients[side] = command_client
                self._required_actions.append((command_name, command_client))

                if self.execute and not self.use_fake_hardware:
                    homing_name = f"/{side}_franka_gripper/homing"
                    homing_client = ActionClient(self, Homing, homing_name)
                    self._gripper_homing_clients[side] = homing_client
                    self._required_actions.append((homing_name, homing_client))

        if self.execute:
            self._add_execute_task_solution_client()

    def _add_execute_task_solution_client(self) -> None:
        try:
            action_module = importlib.import_module(
                "moveit_task_constructor_msgs.action"
            )
            action_type = getattr(action_module, "ExecuteTaskSolution")
        except (ImportError, AttributeError) as exc:
            raise RuntimeError(
                "moveit_task_constructor_msgs/action/ExecuteTaskSolution is "
                "required when execute=true"
            ) from exc

        action_name = "/execute_task_solution"
        self._required_actions.append(
            (action_name, ActionClient(self, action_type, action_name))
        )

    def _joint_state_callback(self, message: JointState) -> None:
        self._joint_names = set(message.name)
        self._joint_state_received_at = time.monotonic()

    def _controller_state_callback(self, side: str) -> None:
        self._controller_state_received_at[side] = time.monotonic()

    def _missing_actions(self) -> list[str]:
        return [
            action_name
            for action_name, client in self._required_actions
            if not client.server_is_ready()
        ]

    def _state_errors(self, now: float) -> list[str]:
        errors: list[str] = []
        missing_joints = missing_arm_joint_names(self._joint_names)
        if missing_joints:
            errors.append(
                "missing arm joints on /joint_states: "
                + ", ".join(sorted(missing_joints))
            )
        elif now - self._joint_state_received_at > self.state_max_age:
            errors.append("/joint_states is stale")

        for side in ARM_SIDES:
            received_at = self._controller_state_received_at[side]
            if received_at <= 0.0:
                errors.append(f"{side} controller_state has not been received")
            elif now - received_at > self.state_max_age:
                errors.append(f"{side} controller_state is stale")
        return errors

    def _wait_for_system_ready(self, deadline: float) -> bool:
        next_report_at = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            missing_actions = self._missing_actions()
            state_errors = self._state_errors(now)
            if not missing_actions and not state_errors:
                return True

            if now >= next_report_at:
                if missing_actions:
                    self.get_logger().info(
                        "Waiting for action servers: "
                        + ", ".join(missing_actions)
                    )
                for state_error in state_errors:
                    self.get_logger().info(f"Waiting for state: {state_error}")
                next_report_at = now + 2.0
        return False

    def _wait_for_future(self, future, deadline: float) -> bool:
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def _home_gripper(self, side: str, deadline: float) -> bool:
        client = self._gripper_homing_clients[side]
        self.get_logger().warning(
            f"{side} gripper Homing is starting; "
            "make sure the gripper is empty"
        )
        goal_future = client.send_goal_async(Homing.Goal())
        if not self._wait_for_future(goal_future, deadline):
            self.get_logger().error(f"{side} gripper Homing goal timed out")
            return False
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{side} gripper Homing goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, deadline):
            self.get_logger().error(f"{side} gripper Homing result timed out")
            return False
        wrapped_result = result_future.result()
        if wrapped_result is None or not bool(wrapped_result.result.success):
            error = "no result"
            if wrapped_result is not None:
                error = wrapped_result.result.error
            self.get_logger().error(
                f"{side} gripper Homing failed: {error}"
            )
            return False
        self.get_logger().info(f"{side} gripper Homing succeeded")
        return True

    def _ensure_grippers_homed(self, deadline: float) -> bool:
        if not self.execute:
            return True
        if not self.start_gripper:
            self.get_logger().error(
                "execute=true requires start_gripper=true because the task "
                "contains gripper stages"
            )
            return False
        if self.use_fake_hardware:
            return True
        if self.home_grippers_before_execute:
            for side in ARM_SIDES:
                if not self._home_gripper(side, deadline):
                    return False
            return True
        if self.grippers_homed:
            self.get_logger().warning(
                "Automatic Homing is disabled; using the operator's "
                "grippers_homed acknowledgement"
            )
            return True

        self.get_logger().error(
            "Real execution requires Homing. Either set "
            "home_grippers_before_execute=true, or home both grippers "
            "manually and set grippers_homed=true."
        )
        return False

    def run(self) -> bool:
        if self.readiness_timeout <= 0.0:
            self.get_logger().error(
                "readiness_timeout must be greater than zero"
            )
            return False
        if self.state_max_age <= 0.0:
            self.get_logger().error("state_max_age must be greater than zero")
            return False

        deadline = time.monotonic() + self.readiness_timeout
        self.get_logger().info(
            "Waiting for both arm controllers and a complete 14-joint state"
        )
        if not self._wait_for_system_ready(deadline):
            self.get_logger().error(
                "Readiness timeout. Missing actions: "
                f"{', '.join(self._missing_actions()) or 'none'}; "
                "state errors: "
                f"{'; '.join(self._state_errors(time.monotonic())) or 'none'}"
            )
            return False

        if not self._ensure_grippers_homed(deadline):
            return False

        # Homing can take time. Re-check that arm state remained live.
        if not self._wait_for_system_ready(deadline):
            self.get_logger().error(
                "Arm state became unavailable after Homing"
            )
            return False

        self.get_logger().info(
            "Readiness checks passed; MTC may now start safely"
        )
        return True


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = TrunkingReadinessGate()
        return 0 if node.run() else 1
    except Exception as exc:  # noqa: BLE001 - launch gate must fail closed
        if node is not None:
            node.get_logger().fatal(f"Readiness gate failed: {exc}")
        else:
            print(f"Readiness gate failed: {exc}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
