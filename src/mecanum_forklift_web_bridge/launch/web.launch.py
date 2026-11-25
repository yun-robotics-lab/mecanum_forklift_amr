import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('mecanum_forklift_web_bridge')
    config = os.path.join(share, 'config', 'web.yaml')
    default_db = os.path.join(get_package_share_directory('mecanum_forklift_docking'), 'config',
                              'dock_database.yaml')
    # '' = no docks
    db = ParameterValue(LaunchConfiguration('dock_database'), value_type=str)
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    address = LaunchConfiguration('address')
    rest_port = ParameterValue(LaunchConfiguration('rest_port'), value_type=int)
    rosbridge_port = ParameterValue(LaunchConfiguration('rosbridge_port'), value_type=int)
    video_port = ParameterValue(LaunchConfiguration('video_port'), value_type=int)
    ns = LaunchConfiguration('topic_ns')       # '' or '/robot1': prefix of the robot's topics
    video_url = ['http://', address, ':', LaunchConfiguration('video_port'),
                 '/stream?topic=', ns, '/camera/image_raw&type=mjpeg&quality=50']
    topics_glob = ["['", ns, "/mission_status', '", ns, "/lift_state', '", ns,
                   "/estop/state']"]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('dock_database', default_value=default_db,
                              description='docks of the running world'),
        DeclareLaunchArgument('address', default_value='127.0.0.1',
                              description='bind address of REST, rosbridge and video'),
        DeclareLaunchArgument('rest_port', default_value='8000'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('video_port', default_value='8080'),
        DeclareLaunchArgument('enable_video', default_value='true'),
        DeclareLaunchArgument('topic_ns', default_value='',
                              description='namespace of the robot (e.g. /robot1) for the '
                                          'rosbridge allow-list and the video URL; the nodes '
                                          'themselves take the namespace pushed by the caller'),
        Node(package='mecanum_forklift_web_bridge', executable='estop_node', name='estop_node',
             output='screen', parameters=[config, {'use_sim_time': use_sim_time}]),
        Node(package='mecanum_forklift_web_bridge', executable='web_api', name='web_backend',
             output='screen',
             parameters=[config, {'dock_database': db, 'use_sim_time': use_sim_time,
                                  'rest_address': address, 'rest_port': rest_port,
                                  'video_url': video_url}]),
        # rosbridge 2.0.4: no authentication, and ONE glob list covers subscribe, advertise and
        # publish together. "[]" = nothing allowed (an empty string would mean unrestricted).
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge_websocket', output='screen',
             parameters=[{'address': address, 'port': rosbridge_port,
                          'topics_glob': ParameterValue(topics_glob, value_type=str),
                          'services_glob': '[]', 'actions_glob': '[]',
                          'use_sim_time': use_sim_time}]),
        Node(package='web_video_server', executable='web_video_server',
             name='web_video_server', output='screen', condition=IfCondition(
                 LaunchConfiguration('enable_video')),
             parameters=[{'address': address, 'port': video_port,
                          'use_sim_time': use_sim_time}]),
    ])
