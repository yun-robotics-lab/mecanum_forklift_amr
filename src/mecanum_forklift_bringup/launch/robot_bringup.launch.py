#!/usr/bin/env python3
"""
Per-robot software stack for 1..N namespaced robots (cf. the reference multiple_bringup_launch.py).

Run after (or together with, see all.launch.py) robot_world.launch.py. It waits until the LAST
robot of the fleet publishes ground-truth odometry (it exists in Gazebo and its bridge works),
then starts one stack per robot, staggered by `robot_stagger_delay` seconds per index so that N
Nav2/AMCL stacks do not all activate at the same instant:
  control (twist_mux, gate, wheel odometry) + lift, localization + Nav2 (initial pose = spawn
  pose), vision, docking, mission manager and the web layer (REST 8000+i, rosbridge 9090+i,
  video 8080+i).
Every robot runs in the namespace /<name> with /tf and /tf_static remapped to <name>/tf ...
(SetRemap), so each robot has its own TF tree. Navigation and localization push the namespace
themselves (their `namespace` argument), everything else through PushRosNamespace.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace, SetRemap

from mecanum_forklift_bringup.fleet import parse_fleet, port_for
from mecanum_forklift_bringup.worlds import dock_database, map_files


def include(package, launch_file, arguments=None, condition=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=(arguments or {}).items(),
                                    condition=condition)


def flag(name):
    return IfCondition(LaunchConfiguration(name))


def launch_setup(context):
    fleet = parse_fleet(LaunchConfiguration('robots').perform(context),
                        LaunchConfiguration('robots_file').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time')
    delay = LaunchConfiguration('robot_stagger_delay')
    address = LaunchConfiguration('address')
    light = LaunchConfiguration('profile').perform(context) == 'light'
    # Resolved now: single_bringup sets its own 'rviz' configuration when it is included.
    rviz = LaunchConfiguration('rviz').perform(context)
    # Maps and docks follow the world; map:= / nav_map:= override the maps.
    world = LaunchConfiguration('world').perform(context)
    share = get_package_share_directory
    amcl_map, nav_map = map_files(world, share('mecanum_forklift_localization'),
                                  share('mecanum_forklift_navigation'))
    amcl_map = LaunchConfiguration('map').perform(context) or amcl_map
    nav_map = LaunchConfiguration('nav_map').perform(context) or nav_map
    if LaunchConfiguration('enable_navigation').perform(context).lower() == 'true':
        for path in (amcl_map, nav_map):
            if not os.path.isfile(path):
                raise RuntimeError(f'world {world}: map not found: {path} (add it and rebuild, '
                                   'or use enable_navigation:=false)')
    docks = dock_database(world, share('mecanum_forklift_docking'))
    notices = [] if docks else [LogInfo(msg=(
        f'[robot_bringup] world {world} has no dock database: docking and mission are NOT '
        'started'))]
    last = fleet[-1]['name']
    wait_for_last_robot = ExecuteProcess(
        cmd=['bash', '-c',
             f'echo "[robot_bringup] waiting for /{last}/odom_ground_truth '
             f'({len(fleet)} robot(s) spawned in Gazebo)"; '
             f'until timeout 5 ros2 topic echo --once /{last}/odom_ground_truth '
             f'--field header.stamp.sec >/dev/null 2>&1; do sleep 1; done; '
             f'echo "[robot_bringup] /{last} is publishing, starting the robot stacks"'],
        output='screen')

    stacks = []
    for index, robot in enumerate(fleet):
        name = robot['name']
        ns = '/' + name
        sim = {'use_sim_time': use_sim_time}
        group = GroupAction([
            PushRosNamespace(ns),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            include('mecanum_forklift_control', 'control.launch.py', sim),
            include('mecanum_forklift_lift', 'lift.launch.py', sim),
            include('mecanum_forklift_vision', 'vision.launch.py',
                    {**sim, 'debug_image': LaunchConfiguration('debug_image')},
                    flag('enable_vision')),
            *([include('mecanum_forklift_docking', 'docking.launch.py',
                       {**sim, 'dock_database': docks, 'nav_map_default': nav_map},
                       flag('enable_docking')),
               include('mecanum_forklift_task_manager', 'mission.launch.py',
                       {**sim, 'dock_database': docks},
                       flag('enable_mission'))] if docks else []),
            include('mecanum_forklift_web_bridge', 'web.launch.py', {
                **sim, 'topic_ns': ns, 'address': address, 'dock_database': docks or '',
                'rest_port': str(port_for(8000, index)),
                'rosbridge_port': str(port_for(9090, index)),
                'video_port': str(port_for(8080, index)),
                'enable_video': LaunchConfiguration('enable_video')}, flag('enable_web')),
        ])
        nav_args = {
            **sim, 'namespace': ns, 'use_namespace': 'true', 'rviz': rviz,
            'map': amcl_map, 'nav_map': nav_map,
            'set_initial_pose': 'true', 'initial_x': str(robot['x_pose']),
            'initial_y': str(robot['y_pose']), 'initial_yaw': str(robot['yaw']),
            'controller_type': 'mppi_light' if light else LaunchConfiguration('controller_type')}
        if light:       # fewer AMCL particles and a lighter MPPI (see the *_light.yaml files)
            nav_args['amcl_params_file'] = os.path.join(
                get_package_share_directory('mecanum_forklift_localization'), 'config',
                'amcl_light.yaml')
        navigation = include('mecanum_forklift_navigation', 'single_bringup.launch.py', nav_args,
                             flag('enable_navigation'))
        stacks.append(TimerAction(period=PythonExpression([str(index), ' * ', delay]),
                                  actions=[group, navigation]))
    return [*notices, wait_for_last_robot,
            RegisterEventHandler(OnProcessExit(target_action=wait_for_last_robot,
                                               on_exit=stacks))]


def generate_launch_description():
    pkg_bringup = get_package_share_directory('mecanum_forklift_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='simple_warehouse.sdf',
                              description='running world (same as robot_world): selects the maps '
                                          'and docks'),
        DeclareLaunchArgument('map', default_value='',
                              description='AMCL map yaml (default: from the world)'),
        DeclareLaunchArgument('nav_map', default_value='',
                              description='Nav2 costmap map yaml (default: from the world)'),
        DeclareLaunchArgument('robots_file', default_value=os.path.join(
            pkg_bringup, 'config', 'robots.yaml'), description='fleet YAML (same as the world)'),
        DeclareLaunchArgument('robots', default_value='',
                              description='inline fleet list (same as given to the world)'),
        DeclareLaunchArgument('robot_stagger_delay', default_value='10.0',
                              description='seconds added per robot index before its stack starts'),
        DeclareLaunchArgument('controller_type', default_value='mppi',
                              choices=['dwb', 'mppi', 'rotation_shim']),
        DeclareLaunchArgument('address', default_value='127.0.0.1',
                              description='bind address of every REST / rosbridge / video server'),
        DeclareLaunchArgument('profile', default_value='full', choices=['full', 'light'],
                              description='light: lighter AMCL and MPPI (use the same value as '
                                          'robot_world)'),
        DeclareLaunchArgument('enable_navigation', default_value='true',
                              description='localization + Nav2 per robot (the heaviest part)'),
        DeclareLaunchArgument('enable_vision', default_value='true'),
        DeclareLaunchArgument('debug_image', default_value='true',
                              description='publish /<robot>/aruco/image (costs CPU)'),
        DeclareLaunchArgument('enable_docking', default_value='true'),
        DeclareLaunchArgument('enable_mission', default_value='true'),
        DeclareLaunchArgument('enable_web', default_value='true'),
        DeclareLaunchArgument('enable_video', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='open one RViz window per robot '
                                          '(needs enable_navigation:=true)'),
        OpaqueFunction(function=launch_setup),
    ])
