#!/usr/bin/env python3
"""Move the cylinders of warehouse_v2.sdf: their bridge and obstacle_mover. Needs /clock."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('mecanum_forklift_gazebo'), 'config')
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                               value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='obstacle_bridge',
             parameters=[{'config_file': os.path.join(config, 'obstacle_bridge.yaml')}, sim_time],
             output='screen'),
        Node(package='mecanum_forklift_gazebo', executable='obstacle_mover.py',
             name='obstacle_mover',
             parameters=[os.path.join(config, 'dynamic_obstacles.yaml'), sim_time],
             output='screen'),
    ])
