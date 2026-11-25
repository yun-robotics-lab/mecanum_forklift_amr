import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mecanum_forklift_task_manager'), 'config', 'mission.yaml')
    default_db = os.path.join(get_package_share_directory('mecanum_forklift_docking'), 'config',
                              'dock_database.yaml')
    db = LaunchConfiguration('dock_database')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('dock_database', default_value=default_db,
                              description='docks of the running world'),
        Node(package='mecanum_forklift_task_manager', executable='mission_manager',
             name='mission_manager', output='screen',
             parameters=[config, {'dock_database': db, 'use_sim_time': use_sim_time}]),
    ])
