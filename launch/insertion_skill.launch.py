"""Invoke insertion in an existing scene; cable_config must match that scene."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from dual_fr3_moveit_config.maniskill_resources import build_maniskill_resources


def skill_node(context):
    config = LaunchConfiguration('cable_config').perform(context)
    backend = LaunchConfiguration('backend').perform(context)
    if backend == 'real':
        if not config:
            raise ValueError('Real insertion requires cable_config:=/path/to/real_insertion.yaml')
        from dual_fr3_moveit_config.moveit_resources import build_moveit_resources
        resources = build_moveit_resources('dual_fr3.urdf.xacro', {})
    else:
        resources = build_maniskill_resources(scene='trunking_cable', cable_config=config)
    return [Node(
        package='dual_fr3_trunking_mtc', executable='usb_insertion_skill.py',
        prefix='/usr/bin/python3', output='screen',
        parameters=[*resources.as_parameters(), {'use_sim_time': backend != 'real'}],
        arguments=['--cable-config', config, '--backend', backend,
                   '--execute', LaunchConfiguration('execute')],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cable_config', default_value='',
                              description='Same geometry YAML as the running ManiSkill scene'),
        DeclareLaunchArgument('backend', default_value='maniskill', choices=('maniskill', 'real')),
        DeclareLaunchArgument('execute', default_value=PythonExpression([
            "'false' if '", LaunchConfiguration('backend'), "' == 'real' else 'true'"]),
            choices=('true', 'false'), description='Real defaults to planning only; simulation keeps its existing default'),
        OpaqueFunction(function=skill_node),
    ])
