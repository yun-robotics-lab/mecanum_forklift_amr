#!/usr/bin/env python3
"""One command: the world and every robot's stack (robot_world + robot_bringup)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ARGS = ('robots_file', 'robots', 'use_sim_time', 'profile')
WORLD_ARGS = ('world', 'gui', 'dynamic_obstacles')    # robot_world.launch.py only


def generate_launch_description():
    share = get_package_share_directory('mecanum_forklift_bringup')

    def include(name, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
            launch_arguments=arguments.items())

    shared = {a: LaunchConfiguration(a) for a in ARGS}
    return LaunchDescription([
        DeclareLaunchArgument('robots_file', default_value=os.path.join(
            share, 'config', 'robots.yaml')),
        DeclareLaunchArgument('robots', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('profile', default_value='full', choices=['full', 'light']),
        DeclareLaunchArgument('gui', default_value='true',),
        DeclareLaunchArgument('world', default_value='simple_warehouse.sdf',
                              description='world file in mecanum_forklift_gazebo/worlds'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='false',
                              choices=['true', 'false'],
                              description='move the cylinders (world:=warehouse_v2.sdf only)'),
        DeclareLaunchArgument('robot_stagger_delay', default_value='10.0'),
        DeclareLaunchArgument('controller_type', default_value='mppi'),
        DeclareLaunchArgument('enable_web', default_value='true'),
        DeclareLaunchArgument('debug_image', default_value='true'),
        DeclareLaunchArgument('enable_video', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='open one RViz window per robot'),
        include('robot_world.launch.py',
                {**shared, **{a: LaunchConfiguration(a) for a in WORLD_ARGS}}),
        include('robot_bringup.launch.py', {
            **shared, 'world': LaunchConfiguration('world'),
            'robot_stagger_delay': LaunchConfiguration('robot_stagger_delay'),
            'controller_type': LaunchConfiguration('controller_type'),
            'enable_web': LaunchConfiguration('enable_web'),
            'debug_image': LaunchConfiguration('debug_image'),
            'enable_video': LaunchConfiguration('enable_video'),
            'rviz': LaunchConfiguration('rviz')}),
    ])
