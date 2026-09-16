from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Mapping

import yaml


GRIPPER_BACKENDS = ("franka", "gazebo", "maniskill", "fake")
GRIPPER_ACTIONS = ("move", "grasp", "hold")
ACTOR_TO_SIDE = {"leader": "left", "follower": "right"}


def resolve_gripper_backend(simulation_backend: str) -> str:
    """Select the gripper interface from the single robot backend."""
    from .runtime.config import resolve_simulation_backend
    backend = resolve_simulation_backend(simulation_backend)
    return "franka" if backend == "real" else backend


def gripper_command_action_name(side: str, backend: str) -> str:
    if backend not in GRIPPER_BACKENDS:
        raise ValueError(f"unknown gripper backend: {backend!r}")
    action_name = "gripper_cmd" if backend in ("gazebo", "maniskill") else "gripper_action"
    return f"/{side}_franka_gripper/{action_name}"


def gripper_result_succeeded(result: Any, action: str) -> bool:
    """Interpret backend result fields without changing action semantics."""
    if hasattr(result, "success"):
        return bool(result.success)
    if hasattr(result, "reached_goal"):
        # GripperCommand has no distinct grasp action.  A position controller
        # may report contact as stalled before it reaches the requested width;
        # this only completes a grasp command, and never proves object retention.
        # ManiSkill preparation separately checks contact and post-release stability.
        return bool(result.reached_goal) or (
            action == "grasp" and bool(getattr(result, "stalled", False))
        )
    return True


@dataclass(frozen=True)
class GripperSafetyLimits:
    min_width: float = 0.0
    max_width: float = 0.08
    max_speed: float = 0.10
    max_force: float = 40.0
    default_timeout: float = 10.0


@dataclass(frozen=True)
class GripperProfile:
    name: str
    action: str
    width: float
    speed: float
    force: float = 0.0
    epsilon_inner: float = 0.005
    epsilon_outer: float = 0.005
    timeout: float = 10.0

    @property
    def finger_joint_position(self) -> float:
        """Convert total opening width to MoveIt's one-finger position."""
        return self.width / 2.0


@dataclass(frozen=True)
class GripperRequest:
    actor: str
    profile: str
    action_override: str | None = None
    width_override: float | None = None


class GripperProfileRegistry:
    def __init__(
        self,
        profiles: Mapping[str, GripperProfile],
        safety: GripperSafetyLimits,
    ) -> None:
        self._profiles = dict(profiles)
        self.safety = safety
        if not self._profiles:
            raise ValueError("at least one gripper profile is required")
        for profile in self._profiles.values():
            self._validate(profile)

    @classmethod
    def load(cls, path: str | Path) -> "GripperProfileRegistry":
        profile_path = Path(path).expanduser()
        with profile_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}
        if not isinstance(raw, dict):
            raise ValueError("gripper profile file must contain a mapping")

        safety_raw = raw.get("safety", {})
        if not isinstance(safety_raw, dict):
            raise ValueError("gripper safety must be a mapping")
        safety = GripperSafetyLimits(
            min_width=float(safety_raw.get("min_width", 0.0)),
            max_width=float(safety_raw.get("max_width", 0.08)),
            max_speed=float(safety_raw.get("max_speed", 0.10)),
            max_force=float(safety_raw.get("max_force", 40.0)),
            default_timeout=float(safety_raw.get("default_timeout", 10.0)),
        )
        cls._validate_safety(safety)

        profiles_raw = raw.get("profiles", {})
        if not isinstance(profiles_raw, dict):
            raise ValueError("gripper profiles must be a mapping")
        profiles: dict[str, GripperProfile] = {}
        for name, values in profiles_raw.items():
            if not isinstance(values, dict):
                raise ValueError(f"gripper profile {name!r} must be a mapping")
            epsilon = values.get("epsilon", {})
            if not isinstance(epsilon, dict):
                raise ValueError(
                    f"gripper profile {name!r} epsilon must be a mapping"
                )
            profiles[str(name)] = GripperProfile(
                name=str(name),
                action=str(values.get("action", "grasp")),
                width=float(values.get("width", 0.0)),
                speed=float(values.get("speed", 0.0)),
                force=float(values.get("force", 0.0)),
                epsilon_inner=float(epsilon.get("inner", 0.005)),
                epsilon_outer=float(epsilon.get("outer", 0.005)),
                timeout=float(values.get("timeout", safety.default_timeout)),
            )
        return cls(profiles, safety)

    @staticmethod
    def _validate_safety(safety: GripperSafetyLimits) -> None:
        values = (
            safety.min_width,
            safety.max_width,
            safety.max_speed,
            safety.max_force,
            safety.default_timeout,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("gripper safety limits must be finite")
        if safety.min_width < 0.0 or safety.max_width <= safety.min_width:
            raise ValueError("invalid gripper width safety range")
        if safety.max_speed <= 0.0 or safety.max_force <= 0.0:
            raise ValueError("gripper speed and force limits must be positive")
        if safety.default_timeout <= 0.0:
            raise ValueError("gripper default timeout must be positive")

    def _validate(self, profile: GripperProfile) -> None:
        if profile.action not in GRIPPER_ACTIONS:
            raise ValueError(
                f"gripper profile {profile.name!r} has invalid action "
                f"{profile.action!r}"
            )
        values = (
            profile.width,
            profile.speed,
            profile.force,
            profile.epsilon_inner,
            profile.epsilon_outer,
            profile.timeout,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                f"gripper profile {profile.name!r} values must be finite"
            )
        if not self.safety.min_width <= profile.width <= self.safety.max_width:
            raise ValueError(
                f"gripper profile {profile.name!r} width is outside safety limits"
            )
        if profile.action != "hold" and not 0.0 < profile.speed <= self.safety.max_speed:
            raise ValueError(
                f"gripper profile {profile.name!r} speed is outside safety limits"
            )
        if not 0.0 <= profile.force <= self.safety.max_force:
            raise ValueError(
                f"gripper profile {profile.name!r} force is outside safety limits"
            )
        if profile.action == "grasp" and profile.force <= 0.0:
            raise ValueError(
                f"gripper profile {profile.name!r} grasp force must be positive"
            )
        if profile.epsilon_inner < 0.0 or profile.epsilon_outer < 0.0:
            raise ValueError(
                f"gripper profile {profile.name!r} epsilon must be non-negative"
            )
        if profile.timeout <= 0.0:
            raise ValueError(
                f"gripper profile {profile.name!r} timeout must be positive"
            )

    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._profiles))

    def get(self, name: str) -> GripperProfile:
        try:
            return self._profiles[name]
        except KeyError as exc:
            available = ", ".join(self.names())
            raise ValueError(
                f"unknown gripper profile {name!r}; available: {available}"
            ) from exc

    def resolve(self, request: GripperRequest) -> GripperProfile:
        if request.actor not in ACTOR_TO_SIDE:
            raise ValueError(f"unknown gripper actor: {request.actor!r}")
        profile = self.get(request.profile)
        updates: dict[str, Any] = {}
        if request.action_override is not None:
            updates["action"] = request.action_override
        if request.width_override is not None:
            updates["width"] = float(request.width_override)
        resolved = replace(profile, **updates)
        self._validate(resolved)
        return resolved


class GripperController:
    """Reusable profile-based gripper facade with one explicit backend."""

    def __init__(
        self,
        profiles: GripperProfileRegistry,
        backend: str,
        node_name: str = "dual_fr3_trunking_gripper",
        use_sim_time: bool = False,
    ) -> None:
        if backend not in GRIPPER_BACKENDS:
            raise ValueError(f"unknown gripper backend: {backend!r}")

        import rclpy
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        self._rclpy = rclpy
        self.profiles = profiles
        self.backend = backend
        self._context = Context()
        rclpy.init(context=self._context)
        self.node = Node(node_name, context=self._context, parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=use_sim_time),
        ])
        self._executor = SingleThreadedExecutor(context=self._context)
        self._clients: dict[tuple[str, str], Any] = {}

        if backend == "franka":
            from franka_msgs.action import Grasp, Move

            for side in ACTOR_TO_SIDE.values():
                node_prefix = f"/{side}_franka_gripper"
                self._clients[(side, "move")] = ActionClient(
                    self.node,
                    Move,
                    f"{node_prefix}/move",
                )
                self._clients[(side, "grasp")] = ActionClient(
                    self.node,
                    Grasp,
                    f"{node_prefix}/grasp",
                )
        else:
            from control_msgs.action import GripperCommand

            for side in ACTOR_TO_SIDE.values():
                self._clients[(side, "command")] = ActionClient(
                    self.node,
                    GripperCommand,
                    gripper_command_action_name(side, backend),
                )

        self.node.get_logger().info(
            f"GripperController initialized with backend={backend}"
        )

    def _wait_for_result(self, future, timeout):
        if self.backend != "maniskill":
            self._rclpy.spin_until_future_complete(
                self.node, future, executor=self._executor, timeout_sec=timeout)
            return
        # Rope contacts can run far below real time. Keep the profile's timeout
        # in simulation seconds, plus a finite wall watchdog for a stopped clock.
        simulation_start = self.node.get_clock().now().nanoseconds
        wall_deadline = time.monotonic()+max(120., 30.*timeout)
        while not future.done() and self._context.ok():
            elapsed = (self.node.get_clock().now().nanoseconds-simulation_start)*1.e-9
            if elapsed >= timeout or time.monotonic() >= wall_deadline:
                break
            self._rclpy.spin_once(self.node, executor=self._executor, timeout_sec=.1)

    def _send_goal(self, client, goal, timeout: float, label: str):
        if not client.wait_for_server(timeout_sec=min(timeout, 5.0)):
            self.node.get_logger().error(f"{label} action server unavailable")
            return None

        goal_future = client.send_goal_async(goal)
        self._rclpy.spin_until_future_complete(
            self.node,
            goal_future,
            executor=self._executor,
            timeout_sec=timeout,
        )
        if not goal_future.done():
            self.node.get_logger().error(f"{label} goal response timed out")
            return None
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error(f"{label} goal was rejected")
            return None

        result_future = goal_handle.get_result_async()
        self._wait_for_result(result_future, timeout)
        if not result_future.done():
            self.node.get_logger().error(f"{label} result timed out; cancelling")
            goal_handle.cancel_goal_async()
            return None
        return result_future.result()

    def execute(self, request: GripperRequest) -> bool:
        profile = self.profiles.resolve(request)
        side = ACTOR_TO_SIDE[request.actor]
        label = (
            f"{request.actor}/{side} gripper profile={profile.name} "
            f"action={profile.action}"
        )
        if profile.action == "hold":
            self.node.get_logger().info(f"{label}: keeping the current grasp")
            return True

        if self.backend == "franka":
            if profile.action == "move":
                from franka_msgs.action import Move

                goal = Move.Goal()
                goal.width = profile.width
                goal.speed = profile.speed
            else:
                from franka_msgs.action import Grasp

                goal = Grasp.Goal()
                goal.width = profile.width
                goal.speed = profile.speed
                goal.force = profile.force
                goal.epsilon.inner = profile.epsilon_inner
                goal.epsilon.outer = profile.epsilon_outer
            client = self._clients[(side, profile.action)]
        else:
            from control_msgs.action import GripperCommand

            goal = GripperCommand.Goal()
            goal.command.position = profile.finger_joint_position
            goal.command.max_effort = (
                profile.force if profile.action == "grasp" else 0.0
            )
            client = self._clients[(side, "command")]

        self.node.get_logger().info(
            f"Executing {label}: width={profile.width:.4f} m, "
            f"speed={profile.speed:.4f} m/s, force={profile.force:.1f} N"
        )
        wrapped_result = self._send_goal(client, goal, profile.timeout, label)
        if wrapped_result is None:
            return False
        result = wrapped_result.result
        success = gripper_result_succeeded(result, profile.action)
        if not success:
            error = getattr(result, "error", "")
            self.node.get_logger().error(f"{label} failed: {error}")
        return success

    def close(self) -> None:
        if getattr(self, "node", None) is not None:
            self.node.destroy_node()
            self.node = None
        if getattr(self, "_executor", None) is not None:
            self._executor.shutdown()
            self._executor = None
        if getattr(self, "_context", None) is not None and self._context.ok():
            self._context.shutdown()
