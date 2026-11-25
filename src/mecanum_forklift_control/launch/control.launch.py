import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mecanum_forklift_control'), 'config', 'control.yaml')
    mux_config = os.path.join(
        get_package_share_directory('mecanum_forklift_control'), 'config', 'twist_mux.yaml')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # arbitration: cmd_vel_teleop (priority) / cmd_vel_nav_safe -> cmd_vel_mux
        # -> cmd_vel_gate -> cmd_vel
        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=[mux_config, {'use_sim_time': use_sim_time}],
             remappings=[('cmd_vel_out', 'cmd_vel_mux')]),
        Node(package='mecanum_forklift_control', executable='cmd_vel_gate',
             name='cmd_vel_gate', output='screen',
             parameters=[config, {'use_sim_time': use_sim_time}]),
        Node(package='mecanum_forklift_control', executable='wheel_odometry',
             name='wheel_odometry', output='screen',
             parameters=[config, {'use_sim_time': use_sim_time}]),
    ])
