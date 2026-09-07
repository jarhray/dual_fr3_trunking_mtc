from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dual_fr3_trunking_mtc.runtime.config import DEFAULTS, launch_default


def generate_launch_description():
    use_fake_hardware = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value=launch_default(DEFAULTS.use_fake_hardware),
    )
    fake_sensor_commands = DeclareLaunchArgument(
        "fake_sensor_commands",
        default_value=launch_default(DEFAULTS.fake_sensor_commands),
    )
    left_robot_ip = DeclareLaunchArgument(
        "left_robot_ip",
        default_value=launch_default(DEFAULTS.left_robot_ip),
    )
    right_robot_ip = DeclareLaunchArgument(
        "right_robot_ip",
        default_value=launch_default(DEFAULTS.right_robot_ip),
    )
    load_gripper = DeclareLaunchArgument(
        "load_gripper",
        default_value=launch_default(DEFAULTS.load_gripper),
    )
    start_gripper = DeclareLaunchArgument(
        "start_gripper",
        default_value=launch_default(DEFAULTS.start_gripper),
    )
    ee_id = DeclareLaunchArgument(
        "ee_id",
        default_value=launch_default(DEFAULTS.ee_id),
    )
    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value=launch_default(DEFAULTS.use_rviz),
    )
    keypoints_file = DeclareLaunchArgument(
        "keypoints_file",
        default_value=os.path.join(
            get_package_share_directory("dual_fr3_trunking_mtc"), "config", "keypoints.yaml"
        ),
    )
    task_frame = DeclareLaunchArgument(
        "task_frame",
        default_value=launch_default(DEFAULTS.task_frame),
    )
    leader_orientation_direction = DeclareLaunchArgument(
        "leader_orientation_direction",
        default_value=launch_default(DEFAULTS.leader_orientation_direction),
    )
    follower_orientation_direction = DeclareLaunchArgument(
        "follower_orientation_direction",
        default_value=launch_default(DEFAULTS.follower_orientation_direction),
    )
    keypoint_marker_scale = DeclareLaunchArgument(
        "keypoint_marker_scale",
        default_value=launch_default(DEFAULTS.keypoint_marker_scale),
    )
    segment_line_width = DeclareLaunchArgument(
        "segment_line_width",
        default_value=launch_default(DEFAULTS.segment_line_width),
    )
    marker_z_offset = DeclareLaunchArgument(
        "marker_z_offset",
        default_value=launch_default(DEFAULTS.marker_z_offset),
    )
    publish_labels = DeclareLaunchArgument(
        "publish_labels",
        default_value=launch_default(DEFAULTS.publish_labels),
    )

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
                "leader_orientation_direction": LaunchConfiguration(
                    "leader_orientation_direction"
                ),
                "follower_orientation_direction": LaunchConfiguration(
                    "follower_orientation_direction"
                ),
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
            leader_orientation_direction,
            follower_orientation_direction,
            keypoint_marker_scale,
            segment_line_width,
            marker_z_offset,
            publish_labels,
            moveit_demo,
            planner_node,
        ]
    )
