"""Open RViz with the Nav2 view (optionally namespaced). Run it next to a running navigation stack."""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def namespaced_config(path, namespace):
    """Copy the RViz config with its absolute topics moved under the namespace."""
    with open(path) as f:
        config = f.read().replace('Value: /', f'Value: /{namespace}/')
    with tempfile.NamedTemporaryFile('w', prefix=f'{namespace}_', suffix='.rviz',
                                     delete=False) as f:
        f.write(config)
    return f.name


def start_rviz(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if namespace:
        rviz_config = namespaced_config(rviz_config, namespace)
    use_sim_time = LaunchConfiguration('use_sim_time')
    return [Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen')]


def generate_launch_description():
    pkg = get_package_share_directory('mecanum_forklift_navigation')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(pkg, 'rviz', 'nav2_default_view.rviz'),
                              description='Full path to the RViz config file'),
        OpaqueFunction(function=start_rviz),
    ])
