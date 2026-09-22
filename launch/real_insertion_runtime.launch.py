"""Connect the insertion adapter to an existing real robot; never bring up hardware."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def runtime_node(context):
    config = LaunchConfiguration('config').perform(context)
    if not config:
        raise ValueError('An explicit real insertion configuration is required')
    nodes = []
    if LaunchConfiguration('load_executor').perform(context) == 'true':
        executor_config = LaunchConfiguration('executor_config').perform(context)
        if not executor_config or not Path(executor_config).is_file():
            raise ValueError('Loading the inactive executor requires an explicit executor_config YAML file')
        nodes.append(Node(package='controller_manager', executable='spawner',
             arguments=['left_insertion_controller', '--controller-manager', '/left/controller_manager',
                        '--controller-type', 'dual_fr3_moveit_config/JointTargetController',
                        '--param-file', executor_config, '--inactive'], output='screen'))
    arguments = ['--config', config]
    if LaunchConfiguration('allow_execution').perform(context) == 'true':
        arguments.append('--allow-execution')
    nodes.append(Node(package='dual_fr3_trunking_mtc', executable='real_insertion_runtime.py',
                 prefix='/usr/bin/python3', arguments=arguments,
                 parameters=[{'use_sim_time': False}], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=''),
        DeclareLaunchArgument('allow_execution', default_value='false', choices=('true', 'false')),
        DeclareLaunchArgument('load_executor', default_value='false', choices=('true', 'false')),
        DeclareLaunchArgument('executor_config', default_value='',
                              description='Explicit reviewed controller YAML, loaded inactive only'),
        OpaqueFunction(function=runtime_node),
    ])
