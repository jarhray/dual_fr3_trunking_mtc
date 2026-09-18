"""Invoke insertion in an existing scene; cable_config must match that scene."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dual_fr3_moveit_config.maniskill_resources import build_maniskill_resources


def skill_node(context):
    config = LaunchConfiguration('cable_config').perform(context)
    resources = build_maniskill_resources(scene='trunking_cable', cable_config=config)
    return [Node(
        package='dual_fr3_trunking_mtc', executable='usb_insertion_skill.py',
        prefix='/usr/bin/python3', output='screen',
        parameters=[*resources.as_parameters(), {'use_sim_time': True}],
        arguments=['--cable-config', config, '--execute', LaunchConfiguration('execute')],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cable_config', default_value='',
                              description='Same geometry YAML as the running ManiSkill scene'),
        DeclareLaunchArgument('execute', default_value='true', choices=('true', 'false')),
        OpaqueFunction(function=skill_node),
    ])
