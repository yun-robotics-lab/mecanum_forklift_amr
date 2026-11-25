"""Nav2 (planner, controller, smoother, behaviors, BT navigator, waypoint follower, velocity smoother).

Structure follows the Nav2 / autonomous-mini-forklift navigation launch template
(RewrittenYaml params, namespace, respawn, optional composition).
Run the simulation and mecanum_forklift_localization/localization.launch.py (AMCL) separately,
or use single_bringup.launch.py for localization + navigation (+ RViz).

Velocity chain: controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_nav_safe
  (a collision monitor stage will be inserted between the smoother and the mux)
  -> twist_mux (in mecanum_forklift_control; teleop has priority) -> cmd_vel_mux -> cmd_vel_gate -> cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('mecanum_forklift_navigation')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    controller_params_file = LaunchConfiguration('controller_params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['nav_map_server', 'controller_server', 'smoother_server', 'planner_server',
                       'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']

    sim = ParameterValue(use_sim_time, value_type=bool)
    autostart_bool = ParameterValue(autostart, value_type=bool)

    # Map the /tf topics to the namespaced ones (needed when a namespace is used).
    tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    controller_remap = tf_remap + [('cmd_vel', 'cmd_vel_nav')]
    # baseline: the smoother feeds the mux directly (collision monitor not inserted yet)
    smoother_remap = tf_remap + [('cmd_vel', 'cmd_vel_nav'),
                                 ('cmd_vel_smoothed', 'cmd_vel_nav_safe')]
    map_remap = tf_remap + [('map', 'nav_map')]
    # recoveries (spin, backup, wait) publish cmd_vel themselves: send them through the smoother,
    # twist_mux and gate too, or they would bypass the e-stop and the arbitration
    behavior_remap = tf_remap + [('cmd_vel', 'cmd_vel_nav')]

    # costmap layers resolve a relative topic inside the costmap's own namespace, so the scan and
    # curated-map topics are made absolute with the robot namespace ('' -> /scan, /nav_map)
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={'use_sim_time': use_sim_time,
                        'topic': PythonExpression(["'", namespace, "/scan'"]),
                        'map_topic': PythonExpression(["'", namespace, "/nav_map'"])},
        convert_types=True)
    # controller_server section: config/controller_<controller_type>.yaml (dwb | mppi | rotation_shim)
    configured_controller_params = RewrittenYaml(
        source_file=controller_params_file,
        root_key=namespace,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True)
    map_params = [{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': sim}]

    declare_args = [
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
                              description='Params file for all Nav2 nodes'),
        DeclareLaunchArgument('controller_type', default_value='mppi',
                              choices=['dwb', 'mppi', 'mppi_light', 'rotation_shim'],
                              description='Local controller: selects the default '
                                          'controller_params_file (config/controller_<type>.yaml)'),
        DeclareLaunchArgument('controller_params_file',
                              default_value=PathJoinSubstitution([
                                  pkg, 'config',
                                  ['controller_', LaunchConfiguration('controller_type'), '.yaml']]),
                              description='controller_server params (overrides controller_type)'),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(pkg, 'maps', 'warehouse_v2_nav.yaml'),
                              description='Curated navigation map (published on nav_map)'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically startup the Nav2 stack'),
        DeclareLaunchArgument('use_composition', default_value='False',
                              description='Use composed bringup (load into an existing container)'),
        DeclareLaunchArgument('container_name', default_value='nav2_container',
                              description='Name of the existing container to load nodes into'),
        DeclareLaunchArgument('use_respawn', default_value='False',
                              description='Respawn nodes if they crash (non-composed only)'),
        DeclareLaunchArgument('log_level', default_value='info', description='Log level'),
    ]

    def node(package, executable, name, remappings, params=None):
        return Node(
            package=package, executable=executable, name=name, output='screen',
            respawn=use_respawn, respawn_delay=2.0,
            parameters=params if params is not None else [configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings)

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            PushRosNamespace(namespace),
            node('nav2_map_server', 'map_server', 'nav_map_server', map_remap, map_params),
            node('nav2_controller', 'controller_server', 'controller_server', controller_remap,
                 [configured_params, configured_controller_params]),
            node('nav2_smoother', 'smoother_server', 'smoother_server', tf_remap),
            node('nav2_planner', 'planner_server', 'planner_server', tf_remap),
            node('nav2_behaviors', 'behavior_server', 'behavior_server', behavior_remap),
            node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', tf_remap),
            node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', tf_remap),
            node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                 smoother_remap),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': sim, 'autostart': autostart_bool,
                             'node_names': lifecycle_nodes}]),
        ])

    def composable(package, plugin, name, remappings, params=None):
        return ComposableNode(
            package=package, plugin=plugin, name=name,
            parameters=params if params is not None else [configured_params],
            remappings=remappings)

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            PushRosNamespace(namespace),
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    composable('nav2_map_server', 'nav2_map_server::MapServer',
                               'nav_map_server', map_remap, map_params),
                    composable('nav2_controller', 'nav2_controller::ControllerServer',
                               'controller_server', controller_remap,
                               [configured_params, configured_controller_params]),
                    composable('nav2_smoother', 'nav2_smoother::SmootherServer',
                               'smoother_server', tf_remap),
                    composable('nav2_planner', 'nav2_planner::PlannerServer',
                               'planner_server', tf_remap),
                    composable('nav2_behaviors', 'behavior_server::BehaviorServer',
                               'behavior_server', behavior_remap),
                    composable('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator',
                               'bt_navigator', tf_remap),
                    composable('nav2_waypoint_follower', 'nav2_waypoint_follower::WaypointFollower',
                               'waypoint_follower', tf_remap),
                    composable('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother',
                               'velocity_smoother', smoother_remap),
                    composable('nav2_lifecycle_manager', 'nav2_lifecycle_manager::LifecycleManager',
                               'lifecycle_manager_navigation', [],
                               [{'use_sim_time': sim, 'autostart': autostart_bool,
                                 'node_names': lifecycle_nodes}]),
                ]),
        ])

    return LaunchDescription([
        # Line-buffered stdout so logs show up immediately.
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        *declare_args,
        load_nodes,
        load_composable_nodes,
    ])
