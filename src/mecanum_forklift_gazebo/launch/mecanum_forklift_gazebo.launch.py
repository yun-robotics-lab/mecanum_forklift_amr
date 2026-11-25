#!/usr/bin/env python3
import os
import re
import subprocess
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def make_spawn_node(context, xacro_file):
    """Convert the URDF to SDF ourselves and spawn that.

    The URDF->SDF converter drops the `ignition:expressed_in` attribute of <fdir1>. Without it the
    wheels' anisotropic friction direction is fixed to the *spinning* wheel frame and the robot
    cannot strafe, so the attribute is added back (frame = the chassis, like Gazebo's
    mecanum_drive.sdf example) before spawning.
    """
    urdf = xacro.process_file(xacro_file).toxml()
    tmp = tempfile.mkdtemp(prefix='mecanum_forklift_')
    urdf_path = os.path.join(tmp, 'robot.urdf')
    sdf_path = os.path.join(tmp, 'robot.sdf')
    with open(urdf_path, 'w') as f:
        f.write(urdf)

    sdf = subprocess.run(['ign', 'sdf', '-p', urdf_path], check=True,
                         capture_output=True, text=True).stdout
    chassis = re.search(r"<link name='([^']+)'", sdf).group(1)
    sdf = re.sub(r"<model name='([^']+)'>",
                 r"<model name='\1' xmlns:ignition='http://ignitionrobotics.org/schema'>", sdf, count=1)
    sdf = sdf.replace('<fdir1>', f"<fdir1 ignition:expressed_in='{chassis}'>")
    with open(sdf_path, 'w') as f:
        f.write(sdf)

    return [Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_path,
            '-name', 'mecanum_forklift',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )]


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('mecanum_forklift_gazebo')
    pkg_description = get_package_share_directory('mecanum_forklift_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    bridge_config = os.path.join(pkg_gazebo, 'config', 'bridge_config.yaml')
    xacro_file = os.path.join(pkg_description, 'urdf', 'mecanum_forklift.urdf.xacro')

    # Lets Fortress resolve model://mecanum_forklift_description/meshes/... (package:// in the URDF)
    # and the models/ folder of this package.
    resource_paths = [
        os.path.dirname(pkg_description),
        os.path.join(pkg_gazebo, 'models'),
    ]
    existing = os.environ.get('IGN_GAZEBO_RESOURCE_PATH')
    if existing:
        resource_paths.append(existing)
    ign_resource = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.pathsep.join(resource_paths))

    world = LaunchConfiguration('world')
    declare_args = [
        DeclareLaunchArgument('world', default_value='simple_warehouse.sdf',
                              description='World file name inside mecanum_forklift_gazebo/worlds'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Open the Gazebo GUI (false = server only, headless)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the simulation clock'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='false',
                              choices=['true', 'false'],
                              description='Move the cylinders (world:=warehouse_v2.sdf only)'),
    ]
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -v 3 ',
                        PythonExpression(["'' if '", gui, "' == 'true' else '-s '"]),
                        PathJoinSubstitution([pkg_gazebo, 'worlds', world])],
        }.items(),
    )

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': sim_time_param}],
        output='screen',
    )

    spawn_robot = OpaqueFunction(function=make_spawn_node, args=[xacro_file])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': sim_time_param}],
        output='screen',
    )

    pkg_control = get_package_share_directory('mecanum_forklift_control')
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_control, 'launch', 'control.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    pkg_lift = get_package_share_directory('mecanum_forklift_lift')
    lift = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_lift, 'launch', 'lift.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    dynamic_obstacles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'dynamic_obstacles.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('dynamic_obstacles')),
    )

    return LaunchDescription([
        ign_resource,
        *declare_args,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        control,
        lift,
        dynamic_obstacles,
    ])
