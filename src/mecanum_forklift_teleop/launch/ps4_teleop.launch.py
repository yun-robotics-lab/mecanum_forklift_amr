import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mecanum_forklift_teleop'), 'config', 'ps4.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0',
                              description='Joystick index (/dev/input/jsN)'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int),
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            output='screen',
        ),
        Node(
            package='mecanum_forklift_teleop',
            executable='ps4_node',
            name='ps4_node',
            parameters=[config],
            output='screen',
        ),
    ])
