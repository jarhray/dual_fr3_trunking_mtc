from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_fake_hardware = DeclareLaunchArgument("use_fake_hardware", default_value="true")
    fake_sensor_commands = DeclareLaunchArgument("fake_sensor_commands", default_value="true")
    left_robot_ip = DeclareLaunchArgument("left_robot_ip", default_value="172.16.0.2")
    right_robot_ip = DeclareLaunchArgument("right_robot_ip", default_value="172.16.0.3")
    load_gripper = DeclareLaunchArgument("load_gripper", default_value="true")
    start_gripper = DeclareLaunchArgument(
        "start_gripper",
        default_value="true",
    )
    ee_id = DeclareLaunchArgument("ee_id", default_value="franka_hand")
    use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")
    keypoints_file = DeclareLaunchArgument(
        "keypoints_file",
        default_value=os.path.join(
            get_package_share_directory("dual_fr3_trunking_mtc"), "config", "keypoints.yaml"
        ),
    )
    task_frame = DeclareLaunchArgument("task_frame", default_value="left_fr3_link0")
    keypoint_marker_scale = DeclareLaunchArgument("keypoint_marker_scale", default_value="0.02")
    segment_line_width = DeclareLaunchArgument("segment_line_width", default_value="0.015")
    marker_z_offset = DeclareLaunchArgument("marker_z_offset", default_value="0.0")
    publish_labels = DeclareLaunchArgument("publish_labels", default_value="true")

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dual_fr3_moveit_config"), "launch", "demo.launch.py"
            )
        ),
        launch_arguments={
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands"),
            "left_robot_ip": LaunchConfiguration("left_robot_ip"),
            "right_robot_ip": LaunchConfiguration("right_robot_ip"),
            "load_gripper": LaunchConfiguration("load_gripper"),
            "start_gripper": LaunchConfiguration("start_gripper"),
            "ee_id": LaunchConfiguration("ee_id"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    planner_node = Node(
        package="dual_fr3_trunking_mtc",
        executable="trunking_plan_node.py",
        prefix="/usr/bin/python3",
        output="screen",
        parameters=[
            {
                "keypoints_file": LaunchConfiguration("keypoints_file"),
                "task_frame": LaunchConfiguration("task_frame"),
                "keypoint_marker_scale": LaunchConfiguration("keypoint_marker_scale"),
                "segment_line_width": LaunchConfiguration("segment_line_width"),
                "marker_z_offset": LaunchConfiguration("marker_z_offset"),
                "publish_labels": LaunchConfiguration("publish_labels"),
            },
        ],
    )

    return LaunchDescription(
        [
            use_fake_hardware,
            fake_sensor_commands,
            left_robot_ip,
            right_robot_ip,
            load_gripper,
            start_gripper,
            ee_id,
            use_rviz,
            keypoints_file,
            task_frame,
            keypoint_marker_scale,
            segment_line_width,
            marker_z_offset,
            publish_labels,
            moveit_demo,
            planner_node,
        ]
    )
