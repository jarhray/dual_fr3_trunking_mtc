from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from dual_fr3_maniskill.cable.backends import CABLE_SOLVERS
from dual_fr3_moveit_config.moveit_resources import build_moveit_resources
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_resources
from dual_fr3_trunking_mtc.runtime.config import (
    DEFAULTS,
    SIMULATION_BACKENDS,
    launch_default,
)


def declare_argument(name, default, **kwargs):
    return DeclareLaunchArgument(
        name,
        default_value=launch_default(default),
        **kwargs,
    )


def generate_launch_description():
    moveit_package = "dual_fr3_moveit_config"
    trunking_package = "dual_fr3_trunking_mtc"

    fake_sensor_commands = declare_argument(
        "fake_sensor_commands", DEFAULTS.fake_sensor_commands
    )
    left_robot_ip = declare_argument("left_robot_ip", DEFAULTS.left_robot_ip)
    right_robot_ip = declare_argument("right_robot_ip", DEFAULTS.right_robot_ip)
    load_gripper = declare_argument("load_gripper", DEFAULTS.load_gripper)
    start_gripper = declare_argument("start_gripper", DEFAULTS.start_gripper)
    ee_id = declare_argument("ee_id", DEFAULTS.ee_id)
    use_rviz = declare_argument("use_rviz", DEFAULTS.use_rviz)
    simulation_backend = declare_argument(
        "simulation_backend",
        DEFAULTS.simulation_backend,
        choices=SIMULATION_BACKENDS,
        description="Robot execution backend (default: gazebo)",
    )
    maniskill_python = declare_argument(
        "maniskill_python",
        os.environ.get("MANISKILL_PYTHON", str(Path.cwd() / ".venv/bin/python")),
    )
    maniskill_viewer = declare_argument("maniskill_viewer", True)
    maniskill_cable = declare_argument(
        "maniskill_cable",
        True,
        description="Enable the ManiSkill USB contact-grasp preparation scene",
    )
    insertion_enabled = declare_argument("insertion_enabled", True)
    load_cable = declare_argument(
        "load_cable",
        True,
        choices=("true", "false"),
        description="false: omit cable physics, retain original dual-arm MTC trajectories for USB grasp debugging",
    )
    cable_solver = declare_argument(
        "cable_solver",
        "mpm",
        choices=CABLE_SOLVERS,
        description="ManiSkill cable model: MPM or PhysX Rope-Actor capsule chain",
    )
    cable_config = declare_argument("cable_config", "")
    cable_trace_dir = declare_argument(
        "cable_trace_dir",
        "",
        description="Optional Rope-Actor substep trace directory; saves state automatically on failure",
    )
    cable_scene = PythonExpression(
        [
            "'trunking_cable' if '",
            LaunchConfiguration("simulation_backend"),
            "' == 'maniskill' and '",
            LaunchConfiguration("maniskill_cable"),
            "'.lower() in ('true', '1', 'yes', 'on') else 'robot'",
        ]
    )

    backend = LaunchConfiguration("simulation_backend")
    mock_hardware = PythonExpression(["'", backend, "' == 'fake'"])
    use_sim_time = PythonExpression(["'", backend, "' in ('gazebo', 'maniskill')"])
    hardware_condition = IfCondition(
        PythonExpression(
            [
                "'",
                backend,
                "' in ('fake', 'real')",
            ]
        )
    )
    gazebo_condition = IfCondition(PythonExpression(["'", backend, "' == 'gazebo'"]))
    maniskill_condition = IfCondition(
        PythonExpression(
            [
                "'",
                backend,
                "' == 'maniskill'",
            ]
        )
    )
    gz_args = declare_argument("gz_args", DEFAULTS.gz_args)
    gazebo_effort = declare_argument("gazebo_effort", DEFAULTS.gazebo_effort)
    keypoints_file = declare_argument(
        "keypoints_file",
        os.path.join(
            get_package_share_directory(trunking_package),
            "config",
            "keypoints.yaml",
        ),
    )
    task_frame = declare_argument("task_frame", DEFAULTS.task_frame)
    initial_leader_index = declare_argument(
        "initial_leader_index", DEFAULTS.initial_leader_index
    )
    initial_follower_index = declare_argument(
        "initial_follower_index", DEFAULTS.initial_follower_index
    )
    leader_group = declare_argument("leader_group", DEFAULTS.leader_group)
    follower_group = declare_argument("follower_group", DEFAULTS.follower_group)
    leader_ik_frame = declare_argument("leader_ik_frame", DEFAULTS.leader_ik_frame)
    follower_ik_frame = declare_argument(
        "follower_ik_frame", DEFAULTS.follower_ik_frame
    )
    leader_orientation_direction = declare_argument(
        "leader_orientation_direction", DEFAULTS.leader_orientation_direction
    )
    follower_orientation_direction = declare_argument(
        "follower_orientation_direction", DEFAULTS.follower_orientation_direction
    )
    cartesian_step_size = declare_argument(
        "cartesian_step_size", DEFAULTS.cartesian_step_size
    )
    cartesian_jump_threshold = declare_argument(
        "cartesian_jump_threshold",
        DEFAULTS.cartesian_jump_threshold,
        description="Relative joint-space jump factor (> 1); incomplete paths are rejected",
    )
    cartesian_path_tolerance = declare_argument(
        "cartesian_path_tolerance",
        DEFAULTS.cartesian_path_tolerance,
        description="Maximum TCP line deviation or in-place turn drift in meters",
    )
    motion_velocity_scaling = declare_argument(
        "motion_velocity_scaling", DEFAULTS.motion_velocity_scaling
    )
    motion_acceleration_scaling = declare_argument(
        "motion_acceleration_scaling", DEFAULTS.motion_acceleration_scaling
    )
    anchor_max_path_z = declare_argument(
        "anchor_max_path_z",
        DEFAULTS.anchor_max_path_z,
        description=(
            "Maximum TCP z in the keypoint frame during " "direct_move_to_next_anchor"
        ),
    )
    leader_lead_distance = declare_argument(
        "leader_lead_distance", DEFAULTS.leader_lead_distance
    )
    anchor_max_path_length_ratio = declare_argument(
        "anchor_max_path_length_ratio",
        DEFAULTS.anchor_max_path_length_ratio,
        description="Maximum anchor TCP path length / actual start-to-target distance (>= 1)",
    )
    tool_roll = declare_argument("tool_roll", DEFAULTS.tool_roll)
    tool_pitch = declare_argument("tool_pitch", DEFAULTS.tool_pitch)
    preparation_enabled = declare_argument(
        "preparation_enabled", DEFAULTS.preparation_enabled
    )
    preparation_height = declare_argument(
        "preparation_height",
        DEFAULTS.preparation_height,
        description="Vertical approach and synchronized descent distance",
    )
    preparation_interactive = declare_argument(
        "preparation_interactive",
        DEFAULTS.preparation_interactive,
        description="Require keyboard confirmation for gripping and descent",
    )
    preparation_search_arguments = [
        declare_argument(name, getattr(DEFAULTS, name), description=description)
        for name, description in (
            (
                "preparation_ik_candidates",
                "Maximum distinct preparation IK candidates per arm",
            ),
            ("preparation_ik_attempts", "Maximum preparation IK seeds per arm"),
            ("preparation_ik_timeout", "Maximum seconds per preparation IK seed"),
            (
                "preparation_min_joint_distance",
                "Minimum IK candidate joint distance in radians",
            ),
            (
                "preparation_candidate_attempts",
                "Maximum rounds across preparation pairs",
            ),
            (
                "preparation_search_timeout",
                "Search seconds, checked between native solver calls",
            ),
        )
    ]
    gripper_profiles_file = declare_argument(
        "gripper_profiles_file",
        os.path.join(
            get_package_share_directory(trunking_package),
            "config",
            "gripper_profiles.yaml",
        ),
    )
    preparation_leader_gripper_profile = declare_argument(
        "preparation_leader_gripper_profile",
        DEFAULTS.preparation_leader_gripper_profile,
    )
    preparation_follower_gripper_profile = declare_argument(
        "preparation_follower_gripper_profile",
        DEFAULTS.preparation_follower_gripper_profile,
    )
    trajectory_execution_duration_scaling = DeclareLaunchArgument(
        "trajectory_execution_duration_scaling",
        default_value=PythonExpression(
            [
                "'10.0' if '",
                backend,
                "' == 'maniskill' and '",
                LaunchConfiguration("maniskill_cable"),
                "'.lower() in ('true', '1', 'yes', 'on') else '",
                str(DEFAULTS.trajectory_execution_duration_scaling),
                "'",
            ]
        ),
    )
    trajectory_execution_goal_margin = DeclareLaunchArgument(
        "trajectory_execution_goal_margin",
        default_value=PythonExpression(
            [
                "'5.0' if '",
                backend,
                "' == 'maniskill' and '",
                LaunchConfiguration("maniskill_cable"),
                "'.lower() in ('true', '1', 'yes', 'on') else '",
                str(DEFAULTS.trajectory_execution_goal_margin),
                "'",
            ]
        ),
    )
    plan = declare_argument("plan", DEFAULTS.plan)
    execute = declare_argument("execute", DEFAULTS.execute)
    execute_stage_by_stage = declare_argument(
        "execute_stage_by_stage",
        DEFAULTS.execute_stage_by_stage,
        description="Execute cached stages from the successful full plan; recover on failure",
    )
    planning_attempts = declare_argument(
        "planning_attempts",
        DEFAULTS.planning_attempts,
        description="Planning attempts, including pipeline retries within a preparation pair",
    )
    execution_replan_attempts = declare_argument(
        "execution_replan_attempts",
        DEFAULTS.execution_replan_attempts,
        description="Maximum recovery replans of unfinished stages after execution failure",
    )
    readiness_timeout = declare_argument(
        "readiness_timeout", DEFAULTS.readiness_timeout
    )
    state_max_age = declare_argument("state_max_age", DEFAULTS.state_max_age)
    home_grippers_before_execute = declare_argument(
        "home_grippers_before_execute",
        DEFAULTS.home_grippers_before_execute,
    )
    grippers_homed = declare_argument("grippers_homed", DEFAULTS.grippers_homed)
    mtc_keep_alive_sec = declare_argument("mtc_keep_alive_sec", DEFAULTS.keep_alive_sec)

    moveit_share = get_package_share_directory(moveit_package)
    hardware_resources = build_moveit_resources(
        "dual_fr3.urdf.xacro",
        {
            "use_fake_hardware": mock_hardware,
            "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands"),
            "left_robot_ip": LaunchConfiguration("left_robot_ip"),
            "right_robot_ip": LaunchConfiguration("right_robot_ip"),
            "load_left_ros2_control": "true",
            "load_right_ros2_control": "true",
            "load_gripper": LaunchConfiguration("load_gripper"),
            "ee_id": LaunchConfiguration("ee_id"),
        },
    )
    gazebo_resources = build_moveit_resources(
        "dual_fr3.gazebo.urdf.xacro",
        {
            "load_gripper": LaunchConfiguration("load_gripper"),
            "ee_id": LaunchConfiguration("ee_id"),
            "gazebo_effort": LaunchConfiguration("gazebo_effort"),
        },
    )

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "demo.launch.py")
        ),
        launch_arguments={
            "simulation_backend": backend,
            "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands"),
            "left_robot_ip": LaunchConfiguration("left_robot_ip"),
            "right_robot_ip": LaunchConfiguration("right_robot_ip"),
            "load_gripper": LaunchConfiguration("load_gripper"),
            "start_gripper": LaunchConfiguration("start_gripper"),
            "ee_id": LaunchConfiguration("ee_id"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "gz_args": LaunchConfiguration("gz_args"),
            "gazebo_effort": LaunchConfiguration("gazebo_effort"),
            "maniskill_python": LaunchConfiguration("maniskill_python"),
            "maniskill_viewer": LaunchConfiguration("maniskill_viewer"),
            "maniskill_scene": cable_scene,
            "cable_config": LaunchConfiguration("cable_config"),
            "load_cable": LaunchConfiguration("load_cable"),
            "insertion_enabled": LaunchConfiguration("insertion_enabled"),
            "cable_solver": LaunchConfiguration("cable_solver"),
            "cable_trace_dir": LaunchConfiguration("cable_trace_dir"),
            "leader_orientation_direction": LaunchConfiguration(
                "leader_orientation_direction"
            ),
            "trajectory_execution_duration_scaling": LaunchConfiguration(
                "trajectory_execution_duration_scaling"
            ),
            "trajectory_execution_goal_margin": LaunchConfiguration(
                "trajectory_execution_goal_margin"
            ),
            "capabilities": "move_group/ExecuteTaskSolutionCapability",
        }.items(),
    )

    mtc_arguments = [
        "--maniskill-cable",
        LaunchConfiguration("maniskill_cable"),
        "--load-cable",
        LaunchConfiguration("load_cable"),
        "--insertion-enabled",
        LaunchConfiguration("insertion_enabled"),
        "--cable-config",
        LaunchConfiguration("cable_config"),
        "--simulation-backend",
        backend,
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
        "--leader-orientation-direction",
        LaunchConfiguration("leader_orientation_direction"),
        "--follower-orientation-direction",
        LaunchConfiguration("follower_orientation_direction"),
        "--cartesian-step-size",
        LaunchConfiguration("cartesian_step_size"),
        "--cartesian-jump-threshold",
        LaunchConfiguration("cartesian_jump_threshold"),
        "--cartesian-path-tolerance",
        LaunchConfiguration("cartesian_path_tolerance"),
        "--motion-velocity-scaling",
        LaunchConfiguration("motion_velocity_scaling"),
        "--motion-acceleration-scaling",
        LaunchConfiguration("motion_acceleration_scaling"),
        "--anchor-max-path-z",
        LaunchConfiguration("anchor_max_path_z"),
        "--anchor-max-path-length-ratio",
        LaunchConfiguration("anchor_max_path_length_ratio"),
        "--leader-lead-distance",
        LaunchConfiguration("leader_lead_distance"),
        "--tool-roll",
        LaunchConfiguration("tool_roll"),
        "--tool-pitch",
        LaunchConfiguration("tool_pitch"),
        "--preparation-enabled",
        LaunchConfiguration("preparation_enabled"),
        "--preparation-height",
        LaunchConfiguration("preparation_height"),
        "--preparation-interactive",
        LaunchConfiguration("preparation_interactive"),
        "--gripper-profiles-file",
        LaunchConfiguration("gripper_profiles_file"),
        "--preparation-leader-gripper-profile",
        LaunchConfiguration("preparation_leader_gripper_profile"),
        "--preparation-follower-gripper-profile",
        LaunchConfiguration("preparation_follower_gripper_profile"),
        "--plan",
        LaunchConfiguration("plan"),
        "--execute",
        LaunchConfiguration("execute"),
        "--execute-stage-by-stage",
        LaunchConfiguration("execute_stage_by_stage"),
        "--planning-attempts",
        LaunchConfiguration("planning_attempts"),
        "--execution-replan-attempts",
        LaunchConfiguration("execution_replan_attempts"),
        "--keep-alive-sec",
        LaunchConfiguration("mtc_keep_alive_sec"),
    ]

    for argument in preparation_search_arguments:
        mtc_arguments.extend(
            [
                "--" + argument.name.replace("_", "-"),
                LaunchConfiguration(argument.name),
            ]
        )

    def mtc_node(resources, condition):
        return Node(
            package=trunking_package,
            executable="trunking_mtc_prototype.py",
            prefix="/usr/bin/python3",
            output="screen",
            additional_env={
                "TRUNKING_LOG_COLOR": EnvironmentVariable(
                    "TRUNKING_LOG_COLOR",
                    default_value="always",
                ),
            },
            parameters=resources.as_parameters()
            + [
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                }
            ],
            arguments=mtc_arguments,
            condition=condition,
        )

    hardware_mtc_node = mtc_node(
        hardware_resources,
        hardware_condition,
    )
    gazebo_mtc_node = mtc_node(
        gazebo_resources,
        gazebo_condition,
    )

    def make_maniskill_mtc(context):
        resources = build_maniskill_resources(
            scene=cable_scene.perform(context),
            cable_config=LaunchConfiguration("cable_config").perform(context),
            load_gripper=LaunchConfiguration("load_gripper").perform(context),
            ee_id=LaunchConfiguration("ee_id").perform(context),
        )
        return [mtc_node(resources, None)]

    maniskill_mtc_node = OpaqueFunction(
        function=make_maniskill_mtc, condition=maniskill_condition
    )

    readiness_node = Node(
        package=trunking_package,
        executable="trunking_readiness_gate.py",
        prefix="/usr/bin/python3",
        output="screen",
        parameters=[
            {
                "simulation_backend": backend,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "execute": ParameterValue(
                    LaunchConfiguration("execute"), value_type=bool
                ),
                "start_gripper": ParameterValue(
                    LaunchConfiguration("start_gripper"), value_type=bool
                ),
                "home_grippers_before_execute": ParameterValue(
                    LaunchConfiguration("home_grippers_before_execute"),
                    value_type=bool,
                ),
                "grippers_homed": ParameterValue(
                    LaunchConfiguration("grippers_homed"), value_type=bool
                ),
                "readiness_timeout": ParameterValue(
                    LaunchConfiguration("readiness_timeout"), value_type=float
                ),
                "state_max_age": ParameterValue(
                    LaunchConfiguration("state_max_age"), value_type=float
                ),
            }
        ],
    )

    def start_mtc_after_readiness(event, _context):
        if event.returncode == 0:
            return [hardware_mtc_node, gazebo_mtc_node, maniskill_mtc_node]
        return [
            LogInfo(
                msg=(
                    "[ERROR] The dual FR3 readiness gate failed; "
                    "the MTC task will not start."
                )
            ),
            EmitEvent(event=Shutdown(reason="Dual FR3 readiness checks failed.")),
        ]

    start_mtc_when_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=readiness_node,
            on_exit=start_mtc_after_readiness,
        )
    )

    return LaunchDescription(
        [
            fake_sensor_commands,
            left_robot_ip,
            right_robot_ip,
            load_gripper,
            start_gripper,
            ee_id,
            use_rviz,
            simulation_backend,
            maniskill_python,
            maniskill_viewer,
            maniskill_cable,
            load_cable,
            insertion_enabled,
            cable_config,
            cable_solver,
            cable_trace_dir,
            gz_args,
            gazebo_effort,
            keypoints_file,
            task_frame,
            initial_leader_index,
            initial_follower_index,
            leader_group,
            follower_group,
            leader_ik_frame,
            follower_ik_frame,
            leader_orientation_direction,
            follower_orientation_direction,
            cartesian_step_size,
            cartesian_jump_threshold,
            cartesian_path_tolerance,
            motion_velocity_scaling,
            motion_acceleration_scaling,
            anchor_max_path_z,
            anchor_max_path_length_ratio,
            leader_lead_distance,
            tool_roll,
            tool_pitch,
            preparation_enabled,
            preparation_height,
            preparation_interactive,
            *preparation_search_arguments,
            gripper_profiles_file,
            preparation_leader_gripper_profile,
            preparation_follower_gripper_profile,
            trajectory_execution_duration_scaling,
            trajectory_execution_goal_margin,
            plan,
            execute,
            execute_stage_by_stage,
            planning_attempts,
            execution_replan_attempts,
            readiness_timeout,
            state_max_age,
            home_grippers_before_execute,
            grippers_homed,
            mtc_keep_alive_sec,
            moveit_demo,
            start_mtc_when_ready,
            readiness_node,
        ]
    )
