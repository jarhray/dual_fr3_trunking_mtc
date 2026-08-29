from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    moveit_package = "dual_fr3_moveit_config"
    trunking_package = "dual_fr3_trunking_mtc"

    use_fake_hardware = DeclareLaunchArgument("use_fake_hardware", default_value="true")
    fake_sensor_commands = DeclareLaunchArgument("fake_sensor_commands", default_value="true")
    left_robot_ip = DeclareLaunchArgument("left_robot_ip", default_value="172.16.0.2")
    right_robot_ip = DeclareLaunchArgument("right_robot_ip", default_value="172.16.0.3")
    load_gripper = DeclareLaunchArgument("load_gripper", default_value="true")
    ee_id = DeclareLaunchArgument("ee_id", default_value="franka_hand")
    use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")
    use_gazebo = DeclareLaunchArgument("use_gazebo", default_value="false")
    gz_args = DeclareLaunchArgument("gz_args", default_value="empty.sdf -r")
    keypoints_file = DeclareLaunchArgument(
        "keypoints_file",
        default_value=os.path.join(
            get_package_share_directory(trunking_package),
            "config",
            "keypoints.yaml",
        ),
    )
    task_frame = DeclareLaunchArgument("task_frame", default_value="left_fr3_link0")
    initial_leader_index = DeclareLaunchArgument("initial_leader_index", default_value="1")
    initial_follower_index = DeclareLaunchArgument("initial_follower_index", default_value="0")
    leader_group = DeclareLaunchArgument("leader_group", default_value="left_fr3_arm")
    follower_group = DeclareLaunchArgument("follower_group", default_value="right_fr3_arm")
    leader_ik_frame = DeclareLaunchArgument("leader_ik_frame", default_value="left_fr3_hand_tcp")
    follower_ik_frame = DeclareLaunchArgument(
        "follower_ik_frame",
        default_value="right_fr3_hand_tcp",
    )
    motion_velocity_scaling = DeclareLaunchArgument(
        "motion_velocity_scaling",
        default_value="0.2",
    )
    motion_acceleration_scaling = DeclareLaunchArgument(
        "motion_acceleration_scaling",
        default_value="0.2",
    )
    leader_lead_distance = DeclareLaunchArgument(
        "leader_lead_distance",
        default_value="0.10",
    )
    tool_roll = DeclareLaunchArgument(
        "tool_roll",
        default_value="3.141592653589793",
    )
    tool_pitch = DeclareLaunchArgument(
        "tool_pitch",
        default_value="0.0",
    )
    align_initial_poses = DeclareLaunchArgument(
        "align_initial_poses",
        default_value="true",
    )
    trajectory_execution_duration_scaling = DeclareLaunchArgument(
        "trajectory_execution_duration_scaling",
        default_value="10.0",
    )
    trajectory_execution_goal_margin = DeclareLaunchArgument(
        "trajectory_execution_goal_margin",
        default_value="5.0",
    )
    plan = DeclareLaunchArgument("plan", default_value="true")
    execute = DeclareLaunchArgument("execute", default_value="false")
    execute_stage_by_stage = DeclareLaunchArgument(
        "execute_stage_by_stage",
        default_value="true",
    )
    mtc_start_delay = DeclareLaunchArgument("mtc_start_delay", default_value="6.0")
    mtc_keep_alive_sec = DeclareLaunchArgument(
        "mtc_keep_alive_sec",
        default_value="30.0",
    )

    moveit_share = get_package_share_directory(moveit_package)
    urdf_xacro = os.path.join(moveit_share, "config", "dual_fr3.urdf.xacro")
    srdf_xacro = os.path.join(moveit_share, "config", "dual_fr3.srdf.xacro")

    robot_description_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_xacro,
            " use_fake_hardware:=",
            LaunchConfiguration("use_fake_hardware"),
            " fake_sensor_commands:=",
            LaunchConfiguration("fake_sensor_commands"),
            " left_robot_ip:=",
            LaunchConfiguration("left_robot_ip"),
            " right_robot_ip:=",
            LaunchConfiguration("right_robot_ip"),
            " load_gripper:=",
            LaunchConfiguration("load_gripper"),
            " ee_id:=",
            LaunchConfiguration("ee_id"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_config, value_type=str)
    }

    robot_description_semantic_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            srdf_xacro,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_config,
            value_type=str,
        )
    }
    kinematics_yaml = load_yaml(moveit_package, "config/kinematics.yaml")
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(moveit_package, "config/ompl_planning.yaml")
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "demo.launch.py")
        ),
        launch_arguments={
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands"),
            "left_robot_ip": LaunchConfiguration("left_robot_ip"),
            "right_robot_ip": LaunchConfiguration("right_robot_ip"),
            "load_gripper": LaunchConfiguration("load_gripper"),
            "ee_id": LaunchConfiguration("ee_id"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "trajectory_execution_duration_scaling": LaunchConfiguration(
                "trajectory_execution_duration_scaling"
            ),
            "trajectory_execution_goal_margin": LaunchConfiguration(
                "trajectory_execution_goal_margin"
            ),
            "capabilities": "move_group/ExecuteTaskSolutionCapability",
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_gazebo")),
    )

    moveit_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "load_gripper": LaunchConfiguration("load_gripper"),
            "ee_id": LaunchConfiguration("ee_id"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "gz_args": LaunchConfiguration("gz_args"),
            "trajectory_execution_duration_scaling": LaunchConfiguration(
                "trajectory_execution_duration_scaling"
            ),
            "trajectory_execution_goal_margin": LaunchConfiguration(
                "trajectory_execution_goal_margin"
            ),
            "capabilities": "move_group/ExecuteTaskSolutionCapability",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
    )

    mtc_node = Node(
        package=trunking_package,
        executable="trunking_mtc_prototype.py",
        prefix="/usr/bin/python3",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
        ],
        arguments=[
            "--keypoints-file",
            LaunchConfiguration("keypoints_file"),
            "--task-frame",
            LaunchConfiguration("task_frame"),
            "--initial-leader-index",
            LaunchConfiguration("initial_leader_index"),
            "--initial-follower-index",
            LaunchConfiguration("initial_follower_index"),
            "--leader-group",
            LaunchConfiguration("leader_group"),
            "--follower-group",
            LaunchConfiguration("follower_group"),
            "--leader-ik-frame",
            LaunchConfiguration("leader_ik_frame"),
            "--follower-ik-frame",
            LaunchConfiguration("follower_ik_frame"),
            "--motion-velocity-scaling",
            LaunchConfiguration("motion_velocity_scaling"),
            "--motion-acceleration-scaling",
            LaunchConfiguration("motion_acceleration_scaling"),
            "--leader-lead-distance",
            LaunchConfiguration("leader_lead_distance"),
            "--tool-roll",
            LaunchConfiguration("tool_roll"),
            "--tool-pitch",
            LaunchConfiguration("tool_pitch"),
            "--align-initial-poses",
            LaunchConfiguration("align_initial_poses"),
            "--plan",
            LaunchConfiguration("plan"),
            "--execute",
            LaunchConfiguration("execute"),
            "--execute-stage-by-stage",
            LaunchConfiguration("execute_stage_by_stage"),
            "--keep-alive-sec",
            LaunchConfiguration("mtc_keep_alive_sec"),
        ],
    )

    delayed_mtc_node = TimerAction(
        period=LaunchConfiguration("mtc_start_delay"),
        actions=[mtc_node],
    )

    return LaunchDescription(
        [
            use_fake_hardware,
            fake_sensor_commands,
            left_robot_ip,
            right_robot_ip,
            load_gripper,
            ee_id,
            use_rviz,
            use_gazebo,
            gz_args,
            keypoints_file,
            task_frame,
            initial_leader_index,
            initial_follower_index,
            leader_group,
            follower_group,
            leader_ik_frame,
            follower_ik_frame,
            motion_velocity_scaling,
            motion_acceleration_scaling,
            leader_lead_distance,
            tool_roll,
            tool_pitch,
            align_initial_poses,
            trajectory_execution_duration_scaling,
            trajectory_execution_goal_margin,
            plan,
            execute,
            execute_stage_by_stage,
            mtc_start_delay,
            mtc_keep_alive_sec,
            moveit_demo,
            moveit_gazebo,
            delayed_mtc_node,
        ]
    )
