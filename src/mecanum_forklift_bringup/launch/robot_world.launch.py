#!/usr/bin/env python3
"""
World and per-robot spawn and bridge for 1..N namespaced robots.

Follows the reference multiple_robot_world.launch.py. Starts Gazebo once (headless unless
gui:=true) and one /clock bridge; then for every robot of the fleet (config/robots.yaml, or
robots:='[...]'): robot_state_publisher in /<name> (its /tf and /tf_static remapped to
<name>/tf ...: one TF tree per robot), the spawn of the namespaced model, and a parameter_bridge
whose topics are /<name>/cmd_vel, /<name>/scan, ... Spawns are chained one after the other
(OnProcessExit) so the models do not race in the physics world.
Run robot_bringup.launch.py in a second terminal (or use all.launch.py, which does both).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mecanum_forklift_bringup.fleet import bridge_arguments, parse_fleet
from mecanum_forklift_bringup.sdf_tools import build_sdf, robot_description

TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def launch_setup(context):
    fleet = parse_fleet(LaunchConfiguration('robots').perform(context),
                        LaunchConfiguration('robots_file').perform(context))
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    profile = LaunchConfiguration('profile').perform(context)
    xacro_file = os.path.join(get_package_share_directory('mecanum_forklift_description'),
                              'urdf', 'mecanum_forklift.urdf.xacro')
    actions = []
    previous_spawn = None
    for robot in fleet:
        name = robot['name']
        namespace = '/' + name
        state_publisher = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', namespace=namespace, remappings=TF_REMAPS,
            output='screen',
            parameters=[{'robot_description': robot_description(xacro_file, namespace, profile),
                         'use_sim_time': sim_time}])
        spawn = Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-file', build_sdf(xacro_file, namespace, profile), '-name', name,
                       '-x', str(robot['x_pose']), '-y', str(robot['y_pose']),
                       '-z', str(robot['z_pose']), '-Y', str(robot['yaw'])])
        bridge = Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
            namespace=namespace, output='screen',
            arguments=bridge_arguments(name, camera=profile != 'light'),
            parameters=[{'use_sim_time': sim_time}])
        group = [state_publisher, spawn, bridge]
        if previous_spawn is None:
            actions.extend(group)
        else:
            actions.append(RegisterEventHandler(OnProcessExit(target_action=previous_spawn,
                                                              on_exit=group)))
        previous_spawn = spawn
    return actions


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('mecanum_forklift_gazebo')
    pkg_description = get_package_share_directory('mecanum_forklift_description')
    pkg_bringup = get_package_share_directory('mecanum_forklift_bringup')
    resource_path = [os.path.dirname(pkg_description), os.path.join(pkg_gazebo, 'models')]
    if os.environ.get('IGN_GAZEBO_RESOURCE_PATH'):
        resource_path.append(os.environ['IGN_GAZEBO_RESOURCE_PATH'])
    gui = LaunchConfiguration('gui')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [
            '-r -v 3 ', PythonExpression(["'' if '", gui, "' == 'true' else '-s '"]),
            PathJoinSubstitution([pkg_gazebo, 'worlds', LaunchConfiguration('world')])]}.items())

    # one clock for the whole world (every world of the gazebo package is named mecanum_world)
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen', arguments=['/world/mecanum_world/clock@rosgraph_msgs/msg/Clock'
                                    '[ignition.msgs.Clock'],
        remappings=[('/world/mecanum_world/clock', '/clock')])

    dynamic_obstacles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'dynamic_obstacles.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('dynamic_obstacles')))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='simple_warehouse.sdf'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='open the Gazebo GUI (false = headless server)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='false',
                              choices=['true', 'false'],
                              description='move the cylinders (world:=warehouse_v2.sdf only)'),
        DeclareLaunchArgument('robots_file', default_value=os.path.join(
            pkg_bringup, 'config', 'robots.yaml'), description='fleet YAML (name, x/y/z, yaw)'),
        DeclareLaunchArgument('robots', default_value='',
                              description='inline fleet list, overrides robots_file'),
        DeclareLaunchArgument('profile', default_value='full', choices=['full', 'light'],
                              description='light: lidar 180 samples at 5 Hz, no camera sensor'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.pathsep.join(resource_path)),
        gz_sim,
        clock_bridge,
        dynamic_obstacles,
        OpaqueFunction(function=launch_setup),
    ])
