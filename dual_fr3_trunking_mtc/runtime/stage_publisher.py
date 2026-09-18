from __future__ import annotations

import logging
from typing import Sequence

from std_msgs.msg import String

from dual_fr3_trunking_mtc.stages.specs import (
    MtcStageSpec,
    mtc_stage_sequence_to_json,
    mtc_stage_sequence_to_text,
)


def start_stage_sequence_publisher(
    specs: Sequence[MtcStageSpec],
    logger: logging.Logger,
):
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
    except ImportError as exc:
        logger.warning("rclpy is not importable; stage sequence topic skipped: %s", exc)
        return None

    initialized_here = False
    interface_node = None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True

        interface_node = rclpy.create_node(
            "dual_fr3_trunking_mtc_prototype_interface"
        )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        json_pub = interface_node.create_publisher(
            String,
            "/dual_fr3_trunking_mtc_prototype/stage_sequence",
            latched_qos,
        )
        text_pub = interface_node.create_publisher(
            String,
            "/dual_fr3_trunking_mtc_prototype/stage_sequence_text",
            latched_qos,
        )

        json_msg = String()
        json_msg.data = mtc_stage_sequence_to_json(specs)
        text_msg = String()
        text_msg.data = mtc_stage_sequence_to_text(specs)
        json_pub.publish(json_msg)
        text_pub.publish(text_msg)
        rclpy.spin_once(interface_node, timeout_sec=0.1)
        logger.info(
            "published MTC stage sequence to "
            "/dual_fr3_trunking_mtc_prototype/stage_sequence"
        )
        return (rclpy, interface_node, initialized_here)
    except Exception as exc:
        logger.warning("stage sequence topic publish skipped: %s", exc)
        if interface_node is not None:
            interface_node.destroy_node()
        if initialized_here:
            rclpy.shutdown()
        return None


def spin_stage_sequence_publisher(handle, timeout_sec: float = 0.1) -> None:
    if handle is None:
        return
    rclpy, interface_node, _initialized_here = handle
    rclpy.spin_once(interface_node, timeout_sec=timeout_sec)


def shutdown_stage_sequence_publisher(handle) -> None:
    if handle is None:
        return
    rclpy, interface_node, initialized_here = handle
    interface_node.destroy_node()
    if initialized_here:
        rclpy.shutdown()
