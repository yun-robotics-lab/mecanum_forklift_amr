"""Fleet loading and per-robot helpers (pure Python, no ROS): used by both launch files."""
import yaml

DEFAULTS = {'z_pose': 0.05, 'yaw': 0.0}


def parse_fleet(robots_arg, robots_file):
    """Return the validated fleet: the inline YAML/JSON `robots_arg` wins over `robots_file`."""
    if robots_arg and robots_arg.strip():
        data = yaml.safe_load(robots_arg)
    else:
        with open(robots_file, encoding='utf-8') as f:
            data = yaml.safe_load(f)
    robots = data['robots'] if isinstance(data, dict) else data
    if not isinstance(robots, list) or not robots:
        raise RuntimeError(f'the fleet must be a non-empty list of robots, got {robots!r}')
    names = set()
    fleet = []
    for entry in robots:
        if not isinstance(entry, dict) or 'name' not in entry:
            raise RuntimeError(f'every robot needs a name and a pose, got {entry!r}')
        robot = {**DEFAULTS, **entry}
        name = str(robot['name'])
        if not name.replace('_', '').isalnum() or not name[0].isalpha():
            raise RuntimeError(f'invalid robot name {name!r} (letters, digits, underscore)')
        if name in names:
            raise RuntimeError(f'duplicate robot name {name!r}')
        names.add(name)
        fleet.append({'name': name, 'x_pose': float(robot['x_pose']),
                      'y_pose': float(robot['y_pose']), 'z_pose': float(robot['z_pose']),
                      'yaw': float(robot['yaw'])})
    return fleet


def port_for(base, index):
    """Return the port of robot `index` (0-based): base + index (REST 8000, rosbridge 9090 ...)."""
    return base + index


# gz topic suffix, ROS message type, Gazebo message type, direction ('[' gz->ros, ']' ros->gz)
BRIDGED = (
    ('cmd_vel', 'geometry_msgs/msg/Twist', 'ignition.msgs.Twist', ']'),
    ('odom_ground_truth', 'nav_msgs/msg/Odometry', 'ignition.msgs.Odometry', '['),
    ('joint_states', 'sensor_msgs/msg/JointState', 'ignition.msgs.Model', '['),
    ('fork/cmd_pos', 'std_msgs/msg/Float64', 'ignition.msgs.Double', ']'),
    ('scan', 'sensor_msgs/msg/LaserScan', 'ignition.msgs.LaserScan', '['),
    ('camera/image_raw', 'sensor_msgs/msg/Image', 'ignition.msgs.Image', '['),
    ('camera/camera_info', 'sensor_msgs/msg/CameraInfo', 'ignition.msgs.CameraInfo', '['),
    ('imu', 'sensor_msgs/msg/Imu', 'ignition.msgs.IMU', '['),
)


def bridge_arguments(name, camera=True):
    """Return the parameter_bridge arguments of robot `name` (topics /<name>/...)."""
    return [f'/{name}/{topic}@{ros}{direction}{gz}' for topic, ros, gz, direction in BRIDGED
            if camera or not topic.startswith('camera/')]
