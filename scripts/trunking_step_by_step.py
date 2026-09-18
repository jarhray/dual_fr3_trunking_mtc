#!/usr/bin/python3

import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

from dual_fr3_trunking_mtc.task.models import (
    DEFAULT_FOLLOWER_ORIENTATION_DIRECTION,
    DEFAULT_LEADER_LEAD_DISTANCE,
    DEFAULT_LEADER_ORIENTATION_DIRECTION,
    ORIENTATION_DIRECTIONS,
    path_orientation_yaw,
)
from dual_fr3_trunking_mtc.task.planner import load_keypoints


TOOL_ROLL = math.pi
TOOL_PITCH = 0.0


def load_controller_class():
    package_prefix = Path(get_package_prefix("dual_fr3_moveit_config"))
    controller_directory = package_prefix / "lib" / "dual_fr3_moveit_config"
    sys.path.insert(0, str(controller_directory))
    from fr3_controller_lin import DualFR3LinearController

    return DualFR3LinearController


def yaw_between(start, goal, orientation_direction):
    return path_orientation_yaw(
        goal.position[0] - start.position[0],
        goal.position[1] - start.position[1],
        orientation_direction,
    )


def start_gazebo():
    print("Starting dual_fr3_moveit_config Gazebo...")
    process = subprocess.Popen(
        ["ros2", "launch", "dual_fr3_moveit_config", "gazebo.launch.py"],
        start_new_session=True,
    )
    time.sleep(5.0)
    return process


def stop_gazebo(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        process.terminate()


def move_ahead(
    controller,
    arm,
    start,
    goal,
    lead_distance,
    orientation_direction,
):
    current = controller.get_current_pose(arm)
    if current is None:
        controller.get_logger().error(f"No current TCP pose for {arm}")
        return False

    delta_x = goal.position[0] - start.position[0]
    delta_y = goal.position[1] - start.position[1]
    delta_z = goal.position[2] - start.position[2]
    distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

    return controller.move_to_cartesian(
        arm,
        current.position.x + lead_distance * delta_x / distance,
        current.position.y + lead_distance * delta_y / distance,
        current.position.z + lead_distance * delta_z / distance,
        TOOL_ROLL,
        TOOL_PITCH,
        yaw_between(start, goal, orientation_direction),
        frame_id=start.frame_id,
    )


def rotate_toward(
    controller,
    arm,
    frame_id,
    start,
    goal,
    orientation_direction,
):
    current = controller.get_current_pose(arm)
    if current is None:
        controller.get_logger().error(f"No current TCP pose for {arm}")
        return False

    return controller.move_to_cartesian(
        arm,
        current.position.x,
        current.position.y,
        current.position.z,
        TOOL_ROLL,
        TOOL_PITCH,
        yaw_between(start, goal, orientation_direction),
        frame_id=frame_id,
    )


def run_actions(actions):
    print("\nActions:")
    for index, (description, _) in enumerate(actions):
        print(f"  [{index:02d}] {description}")

    for index, (description, action) in enumerate(actions):
        answer = input(
            f"\n[{index:02d}] {description}\n"
            "Press Enter to run, s to skip, q to quit: "
        ).strip().lower()
        if answer == "q":
            return True
        if answer == "s":
            continue
        if not action():
            print(f"FAILED [{index:02d}]: {description}")
            return False
        print(f"SUCCESS [{index:02d}]")
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--no-gazebo",
        action="store_true",
        help="Use an already running dual_fr3_moveit_config Gazebo session.",
    )
    parser.add_argument(
        "--leader-lead-distance",
        type=float,
        default=DEFAULT_LEADER_LEAD_DISTANCE,
        help="Leader clearance distance in metres (default: 0.10).",
    )
    parser.add_argument(
        "--leader-orientation-direction",
        choices=ORIENTATION_DIRECTIONS,
        default=DEFAULT_LEADER_ORIENTATION_DIRECTION,
        help="Leader TCP path-facing direction (default: reverse).",
    )
    parser.add_argument(
        "--follower-orientation-direction",
        choices=ORIENTATION_DIRECTIONS,
        default=DEFAULT_FOLLOWER_ORIENTATION_DIRECTION,
        help="Follower TCP path-facing direction (default: forward).",
    )
    args = parser.parse_args()
    if args.leader_lead_distance <= 0.0:
        parser.error("--leader-lead-distance must be greater than zero")
    lead_distance_cm = f"{args.leader_lead_distance * 100:g}"

    keypoints_file = (
        Path(get_package_share_directory("dual_fr3_trunking_mtc"))
        / "config"
        / "keypoints.yaml"
    )
    entry_0, corner_1, corner_2, corner_3, corner_4, entry_5 = load_keypoints(
        keypoints_file
    )[:6]

    gazebo_process = None
    controller = None
    executor = None

    try:
        if not args.no_gazebo:
            gazebo_process = start_gazebo()

        rclpy.init()
        controller_class = load_controller_class()
        controller = controller_class(
            node_name="dual_fr3_trunking_step_by_step",
            default_frame=entry_0.frame_id,
        )

        executor = MultiThreadedExecutor()
        executor.add_node(controller)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        actions = [
            (
                "follower: close gripper (initialization)",
                lambda: controller.close_gripper("right"),
            ),
            (
                "leader: close gripper (initialization)",
                lambda: controller.close_gripper("left"),
            ),
            (
                "follower: current -> entry_0 (OMPL)",
                lambda: controller.move_to_ompl(
                    "right", *entry_0.position,
                    TOOL_ROLL, TOOL_PITCH,
                    yaw_between(
                        entry_0,
                        corner_1,
                        args.follower_orientation_direction,
                    ),
                    frame_id=entry_0.frame_id,
                ),
            ),
            (
                "leader: current -> corner_1 (OMPL)",
                lambda: controller.move_to_ompl(
                    "left", *corner_1.position,
                    TOOL_ROLL, TOOL_PITCH,
                    yaw_between(
                        corner_1,
                        corner_2,
                        args.leader_orientation_direction,
                    ),
                    frame_id=corner_1.frame_id,
                ),
            ),
            (
                f"leader: move {lead_distance_cm} cm toward corner_2 "
                "(Cartesian)",
                lambda: move_ahead(
                    controller,
                    "left",
                    corner_1,
                    corner_2,
                    args.leader_lead_distance,
                    args.leader_orientation_direction,
                ),
            ),
            (
                "follower: entry_0 -> corner_1 (OMPL)",
                lambda: controller.move_to_ompl(
                    "right", *corner_1.position,
                    TOOL_ROLL, TOOL_PITCH,
                    yaw_between(
                        corner_1,
                        corner_2,
                        args.follower_orientation_direction,
                    ),
                    frame_id=corner_1.frame_id,
                ),
            ),
            (
                "leader: lead pose -> entry_5 (OMPL)",
                lambda: controller.move_to_ompl(
                    "left", *entry_5.position,
                    TOOL_ROLL, TOOL_PITCH,
                    yaw_between(
                        corner_4,
                        entry_5,
                        args.leader_orientation_direction,
                    ),
                    frame_id=entry_5.frame_id,
                ),
            ),
            (
                "follower: corner_1 -> corner_2 (Cartesian)",
                lambda: controller.move_to_cartesian(
                    "right", *corner_2.position,
                    TOOL_ROLL, TOOL_PITCH,
                    yaw_between(
                        corner_1,
                        corner_2,
                        args.follower_orientation_direction,
                    ),
                    frame_id=corner_2.frame_id,
                ),
            ),
            (
                "follower: rotate toward corner_3 (Cartesian)",
                lambda: rotate_toward(
                    controller,
                    "right",
                    corner_2.frame_id,
                    corner_2,
                    corner_3,
                    args.follower_orientation_direction,
                ),
            ),
            (
                "follower: corner_2 -> corner_3 (Cartesian)",
                lambda: controller.move_to_cartesian(
                    "right", *corner_3.position,
                    TOOL_ROLL, TOOL_PITCH,
                    yaw_between(
                        corner_2,
                        corner_3,
                        args.follower_orientation_direction,
                    ),
                    frame_id=corner_3.frame_id,
                ),
            ),
            (
                "follower: rotate toward corner_4 (Cartesian)",
                lambda: rotate_toward(
                    controller,
                    "right",
                    corner_3.frame_id,
                    corner_3,
                    corner_4,
                    args.follower_orientation_direction,
                ),
            ),
            (
                "follower: corner_3 -> corner_4 (Cartesian)",
                lambda: controller.move_to_cartesian(
                    "right", *corner_4.position,
                    TOOL_ROLL, TOOL_PITCH,
                    yaw_between(
                        corner_3,
                        corner_4,
                        args.follower_orientation_direction,
                    ),
                    frame_id=corner_4.frame_id,
                ),
            ),
            (
                f"leader: move {lead_distance_cm} cm beyond entry_5 "
                "(Cartesian)",
                lambda: move_ahead(
                    controller,
                    "left",
                    corner_4,
                    entry_5,
                    args.leader_lead_distance,
                    args.leader_orientation_direction,
                ),
            ),
            (
                "follower: corner_4 -> entry_5 (OMPL)",
                lambda: controller.move_to_ompl(
                    "right", *entry_5.position,
                    TOOL_ROLL, TOOL_PITCH,
                    yaw_between(
                        corner_4,
                        entry_5,
                        args.follower_orientation_direction,
                    ),
                    frame_id=entry_5.frame_id,
                ),
            ),
        ]

        return 0 if run_actions(actions) else 1
    except (KeyboardInterrupt, RuntimeError) as error:
        print(f"\nStopped: {error}")
        return 130 if isinstance(error, KeyboardInterrupt) else 1
    finally:
        if controller is not None:
            controller.destroy_node()
        if executor is not None:
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        stop_gazebo(gazebo_process)


if __name__ == "__main__":
    raise SystemExit(main())
