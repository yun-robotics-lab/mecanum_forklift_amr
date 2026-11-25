"""Start map_server + AMCL (omni motion model) + lifecycle manager. Run the simulation separately.

Structure follows the Nav2 / autonomous-mini-forklift localization_launch.py template
(RewrittenYaml params, namespace, respawn, optional composition).

Do NOT run together with mapping.launch.py: AMCL and slam_toolbox must not both publish map->odom.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('mecanum_forklift_localization')
    rviz_config = os.path.join(pkg, 'config', 'localization.rviz')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server', 'amcl']

    # Map the /tf topics to the namespaced ones (needed when a namespace is used).
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    sim = ParameterValue(use_sim_time, value_type=bool)
    flt = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)  # noqa: E731
    initial_pose = {
        'set_initial_pose': ParameterValue(LaunchConfiguration('set_initial_pose'),
                                           value_type=bool),
        'initial_pose': {'x': flt('initial_x'), 'y': flt('initial_y'), 'z': 0.0,
                         'yaw': flt('initial_yaw')},
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={'use_sim_time': use_sim_time,
                        'yaml_filename': map_yaml_file},
        convert_types=True)

    declare_args = [
        DeclareLaunchArgument('namespace', default_value='',
                              description='Top-level namespace'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'simple_warehouse.yaml'),
                              description='Full path to the map yaml file'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'amcl.yaml'),
                              description='Params file for map_server and amcl'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically startup the localization stack'),
        DeclareLaunchArgument('use_composition', default_value='False',
                              description='Use composed bringup (load into an existing container)'),
        DeclareLaunchArgument('container_name', default_value='nav2_container',
                              description='Name of the existing container to load nodes into'),
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

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            PushRosNamespace(namespace),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, initial_pose],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': sim,
                             'autostart': ParameterValue(autostart, value_type=bool),
                             'node_names': lifecycle_nodes}]),
        ])

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[PushRosNamespace(namespace), LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params, initial_pose],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': sim,
                             'autostart': ParameterValue(autostart, value_type=bool),
                             'node_names': lifecycle_nodes}]),
        ])])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_localization',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': sim}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log')

    return LaunchDescription([
        # Line-buffered stdout so logs show up immediately.
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        *declare_args,
        load_nodes,
        load_composable_nodes,
        # rviz,
    ])
