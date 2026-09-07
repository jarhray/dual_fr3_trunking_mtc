from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from .markers import build_marker_array
from .models import TaskPlan, TaskStep
from .planner import build_segment_plans, load_keypoints, plan_to_dict
from .runtime.config import DEFAULTS
from .scheduler import build_task_plan


class TrunkingPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("dual_fr3_trunking_planner")

        default_keypoints = str(
            Path(get_package_share_directory("dual_fr3_trunking_mtc"))
            / "config"
            / "keypoints.yaml"
        )

        self.declare_parameter("keypoints_file", default_keypoints)
        self.declare_parameter("task_frame", DEFAULTS.task_frame)
        self.declare_parameter("samples_per_segment", DEFAULTS.samples_per_segment)
        self.declare_parameter("publish_markers", DEFAULTS.publish_markers)
        self.declare_parameter("auto_reload", DEFAULTS.auto_reload)
        self.declare_parameter("reload_period_sec", DEFAULTS.reload_period_sec)
        self.declare_parameter(
            "keypoint_marker_scale",
            DEFAULTS.keypoint_marker_scale,
        )
        self.declare_parameter("segment_line_width", DEFAULTS.segment_line_width)
        self.declare_parameter("marker_z_offset", DEFAULTS.marker_z_offset)
        self.declare_parameter("publish_labels", DEFAULTS.publish_labels)
        self.declare_parameter(
            "initial_leader_index",
            DEFAULTS.initial_leader_index,
        )
        self.declare_parameter(
            "initial_follower_index",
            DEFAULTS.initial_follower_index,
        )
        self.declare_parameter(
            "leader_orientation_direction",
            DEFAULTS.leader_orientation_direction,
        )
        self.declare_parameter(
            "follower_orientation_direction",
            DEFAULTS.follower_orientation_direction,
        )

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.summary_pub = self.create_publisher(String, "~/plan_summary", latched_qos)
        self.schedule_pub = self.create_publisher(String, "~/task_schedule", latched_qos)
        self.markers_pub = self.create_publisher(MarkerArray, "~/markers", latched_qos)
        self.replan_srv = self.create_service(Trigger, "~/replan", self._on_replan)

        self._last_keypoints_file: Optional[str] = None
        self._last_mtime: Optional[float] = None
        self._keypoints = []
        self._task_plan = TaskPlan((), (), (), 1, 0)

        self._load_and_publish(force=True)

        if self.get_parameter("auto_reload").value:
            period = float(self.get_parameter("reload_period_sec").value)
            self._timer = self.create_timer(period, self._maybe_reload)
        else:
            self._timer = None

    def _keypoints_path(self) -> Path:
        return Path(self.get_parameter("keypoints_file").value).expanduser()

    def _load_and_publish(self, force: bool = False) -> bool:
        keypoints_path = self._keypoints_path()
        if not keypoints_path.exists():
            self.get_logger().error(f"Keypoints file not found: {keypoints_path}")
            return False

        mtime = keypoints_path.stat().st_mtime
        same_file = self._last_keypoints_file == str(keypoints_path)
        same_mtime = self._last_mtime == mtime
        if not force and same_file and same_mtime:
            self._publish_task_schedule_text()
            self._publish_markers()
            return True

        task_frame = str(self.get_parameter("task_frame").value)
        self._keypoints = load_keypoints(keypoints_path, fallback_frame=task_frame)
        samples_per_segment = int(self.get_parameter("samples_per_segment").value)
        leader_orientation_direction = str(
            self.get_parameter("leader_orientation_direction").value
        )
        follower_orientation_direction = str(
            self.get_parameter("follower_orientation_direction").value
        )
        segments = build_segment_plans(
            self._keypoints,
            samples_per_segment=samples_per_segment,
            leader_orientation_direction=leader_orientation_direction,
            follower_orientation_direction=follower_orientation_direction,
        )
        initial_leader_index = int(self.get_parameter("initial_leader_index").value)
        initial_follower_index = int(self.get_parameter("initial_follower_index").value)
        self._task_plan = build_task_plan(
            segments,
            initial_leader_index=initial_leader_index,
            initial_follower_index=initial_follower_index,
        )

        if not self._keypoints:
            self.get_logger().warn(f"No keypoints loaded from {keypoints_path}")
        else:
            self.get_logger().info(
                f"Loaded {len(self._keypoints)} keypoints and "
                f"{len(self._task_plan.segments)} segments from {keypoints_path}"
            )
            self._log_segment_plan()
            self._log_task_schedule()

        self._last_keypoints_file = str(keypoints_path)
        self._last_mtime = mtime
        self._publish_current()
        return True

    def _log_segment_plan(self) -> None:
        for segment in self._task_plan.segments:
            execution_order = " -> ".join(segment.execution_order)
            self.get_logger().info(
                f"Segment {segment.index}: {segment.start.name} -> "
                f"{segment.goal.name}, action={segment.action}, "
                f"path_type={segment.path_type}, "
                f"in_slot={segment.start.in_slot}->{segment.goal.in_slot}, "
                f"execution_order={execution_order}"
            )

    def _log_task_schedule(self) -> None:
        for step in self._task_plan.steps:
            execution_order = " -> ".join(step.execution_order)
            leader_text = self._arm_step_text(
                step.leader_mode,
                step.leader_from_index,
                step.leader_to_index,
                step.leader_hold_index,
            )
            follower_text = self._arm_step_text(
                step.follower_mode,
                step.follower_from_index,
                step.follower_to_index,
                step.follower_hold_index,
            )
            self.get_logger().info(
                f"Task step {step.index}: action={step.action}, "
                f"leader={leader_text}, "
                f"follower={follower_text}, "
                f"execution_order={execution_order}"
            )

    def _arm_step_text(
        self,
        mode: str,
        from_index: int | None,
        to_index: int | None,
        hold_index: int | None,
    ) -> str:
        if hold_index is not None:
            return f"{mode}({self._task_plan.keypoints[hold_index].name})"
        if from_index is not None and to_index is not None:
            return (
                f"{mode}({self._task_plan.keypoints[from_index].name}->"
                f"{self._task_plan.keypoints[to_index].name})"
            )
        return mode

    def _publish_current(self) -> None:
        summary = plan_to_dict(self._task_plan)
        msg = String()
        msg.data = json.dumps(summary, ensure_ascii=False, indent=2)
        self.summary_pub.publish(msg)
        self._publish_task_schedule_text()
        self._publish_markers()

    def _publish_task_schedule_text(self) -> None:
        msg = String()
        msg.data = "\n".join(
            self._task_step_text(step)
            for step in self._task_plan.steps
        )
        self.schedule_pub.publish(msg)

    def _task_step_text(self, step: TaskStep) -> str:
        execution_order = " -> ".join(step.execution_order)
        leader_text = self._arm_step_text(
            step.leader_mode,
            step.leader_from_index,
            step.leader_to_index,
            step.leader_hold_index,
        )
        follower_text = self._arm_step_text(
            step.follower_mode,
            step.follower_from_index,
            step.follower_to_index,
            step.follower_hold_index,
        )
        return (
            f"{step.index}: action={step.action}, "
            f"leader={leader_text}, "
            f"follower={follower_text}, "
            f"order={execution_order}"
        )

    def _publish_markers(self) -> None:
        if bool(self.get_parameter("publish_markers").value):
            markers = build_marker_array(
                self._task_plan.keypoints,
                self._task_plan.segments,
                keypoint_scale=float(
                    self.get_parameter("keypoint_marker_scale").value
                ),
                segment_width=float(self.get_parameter("segment_line_width").value),
                z_offset=float(self.get_parameter("marker_z_offset").value),
                show_labels=bool(self.get_parameter("publish_labels").value),
            )
            self.markers_pub.publish(markers)

    def _maybe_reload(self) -> None:
        self._load_and_publish(force=False)

    def _on_replan(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok = self._load_and_publish(force=True)
        response.success = ok
        response.message = "replanned" if ok else "failed"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrunkingPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
