import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('mecanum_forklift_docking')
    cfg = os.path.join(share, 'config')
    default_db = os.path.join(cfg, 'dock_database.yaml')
    default_nav_map = os.path.join(
        get_package_share_directory('mecanum_forklift_navigation'), 'maps',
        'simple_warehouse_nav.yaml')
    db = LaunchConfiguration('dock_database')
    nav_map = LaunchConfiguration('nav_map_default')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('dock_database', default_value=default_db,
                              description='docks of the running world'),
        DeclareLaunchArgument('nav_map_default', default_value=default_nav_map,
                              description='full Nav2 map, restored after a load/unload'),
        Node(package='opennav_docking', executable='opennav_docking', name='docking_server',
             output='screen',
             parameters=[os.path.join(cfg, 'docking_server.yaml'),
                         {'dock_database': db, 'use_sim_time': use_sim_time}],
             # velocity goes through twist_mux (priority) and cmd_vel_gate, never to cmd_vel
             remappings=[('cmd_vel', 'cmd_vel_dock'),
                         ('detected_dock_pose', 'aruco_dock_pose')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_docking', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['docking_server']}]),
        Node(package='mecanum_forklift_docking', executable='pallet_dock_workflow',
             name='pallet_dock_workflow', output='screen',
             parameters=[os.path.join(cfg, 'workflow.yaml'),
                         {'dock_database': db, 'nav_map_default': nav_map,
                          'use_sim_time': use_sim_time}]),
    ])
