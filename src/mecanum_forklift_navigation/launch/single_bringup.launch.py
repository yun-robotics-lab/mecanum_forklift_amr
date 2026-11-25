"""Bring up localization (map_server + AMCL) and navigation (Nav2) for ONE robot, plus RViz.

Structure follows the autonomous-mini-forklift bringup_launch.py template. Start the simulation
separately (mecanum_forklift_gazebo). multiple_bringup.launch.py (next step) will reuse this file
once per robot namespace.

Maps: `map` is the AMCL map (localization), `nav_map` the curated Nav2 costmap map.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_pkg = get_package_share_directory('mecanum_forklift_navigation')
    loc_pkg = get_package_share_directory('mecanum_forklift_localization')
    nav_launch_dir = os.path.join(nav_pkg, 'launch')
    loc_launch_dir = os.path.join(loc_pkg, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    nav_map_yaml_file = LaunchConfiguration('nav_map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    container_name = 'nav2_container'

    # The children push the namespace themselves, so pass it only when use_namespace is true.
    effective_ns = PythonExpression(["'", namespace, "' if '", use_namespace, "'.lower() == 'true' else ''"])

    container_params = RewrittenYaml(
        source_file=params_file,
        root_key=effective_ns,
        param_rewrites={'use_sim_time': use_sim_time, 'autostart': autostart},
        convert_types=True)

    declare_args = [
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_namespace', default_value='false',
                              description='Whether to apply the namespace to the stack'),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(loc_pkg, 'maps', 'warehouse_v2.yaml'),
                              description='AMCL map yaml file'),
        DeclareLaunchArgument('nav_map',
                              default_value=os.path.join(nav_pkg, 'maps', 'warehouse_v2_nav.yaml'),
                              description='Curated Nav2 costmap map yaml file'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(nav_pkg, 'config', 'nav2_params.yaml'),
                              description='Nav2 params file'),
        DeclareLaunchArgument('controller_type', default_value='mppi',
                              choices=['dwb', 'mppi', 'mppi_light', 'rotation_shim'],
                              description='Local controller (config/controller_<type>.yaml)'),
        DeclareLaunchArgument('amcl_params_file',
                              default_value=os.path.join(loc_pkg, 'config', 'amcl.yaml'),
                              description='map_server + AMCL params file'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically startup the Nav2 stack'),
        DeclareLaunchArgument('use_composition', default_value='False',
                              description='Use composed bringup (one shared container)'),
        DeclareLaunchArgument('use_respawn', default_value='False',
                              description='Respawn nodes if they crash (non-composed only)'),
        DeclareLaunchArgument('log_level', default_value='info', description='Log level'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument('set_initial_pose', default_value='true',
                              description='Give AMCL the initial pose below at startup'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
    ]

    # Shared container (only when composition is enabled); localization and navigation load into it.
    container = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            PushRosNamespace(effective_ns),
            Node(
                name=container_name,
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[container_params, {'autostart': ParameterValue(autostart, value_type=bool)}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                output='screen'),
        ])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(loc_launch_dir, 'localization.launch.py')),
        launch_arguments={
            'namespace': effective_ns,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': amcl_params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': container_name,
            'log_level': log_level,
            'rviz': 'false',  # RViz is started once below
            'set_initial_pose': LaunchConfiguration('set_initial_pose'),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
        }.items())

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'navigation.launch.py')),
        launch_arguments={
            'namespace': effective_ns,
            'map': nav_map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'controller_type': LaunchConfiguration('controller_type'),
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': container_name,
            'log_level': log_level,
        }.items())

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'rviz.launch.py')),
        condition=IfCondition(LaunchConfiguration('rviz')),
        launch_arguments={'namespace': effective_ns, 'use_sim_time': use_sim_time}.items())

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        *declare_args,
        container,
        # Scoped groups: localization and navigation both use `params_file` / `map`; without
        # scoping the first include would leak its values into the second.
        GroupAction(actions=[localization], scoped=True),
        GroupAction(actions=[navigation], scoped=True),
        rviz,
    ])
