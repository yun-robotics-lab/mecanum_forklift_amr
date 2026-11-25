#!/usr/bin/env python3
"""Run the REST API: a ROS backend node spinning in a thread and uvicorn in the main thread."""
import logging
import os
import threading

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import MultiThreadedExecutor
import uvicorn

from mecanum_forklift_web_bridge.api import create_app
from mecanum_forklift_web_bridge.auth import load_token, RateLimiter
from mecanum_forklift_web_bridge.ros_backend import RosBackend


def main(args=None):
    """Create the backend and serve."""
    logging.basicConfig(level=logging.INFO, format='[%(name)s] %(message)s')
    rclpy.init(args=args)
    backend = RosBackend()
    backend.declare_parameter('rest_address', '127.0.0.1')
    backend.declare_parameter('rest_port', 8000)
    backend.declare_parameter('token_file', '~/.config/mecanum_forklift/web_token')
    backend.declare_parameter('cors_origins', [''])
    backend.declare_parameter('map_bounds', [-3.04, -2.54, 3.01, 2.56])
    backend.declare_parameter('max_body', 4096)
    backend.declare_parameter('rate_capacity', 5)
    backend.declare_parameter('rate_per_minute', 10)
    p = backend.get_parameter
    token = load_token(p('token_file').value)
    origins = [o for o in p('cors_origins').value if o]
    app = create_app(
        backend, token, RateLimiter(p('rate_capacity').value, p('rate_per_minute').value),
        tuple(p('map_bounds').value), origins, int(p('max_body').value),
        os.path.join(get_package_share_directory('mecanum_forklift_web_bridge'), 'static'))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(backend)
    threading.Thread(target=executor.spin, daemon=True).start()
    host, port = p('rest_address').value, int(p('rest_port').value)
    backend.get_logger().info(f'REST API on http://{host}:{port} (token file '
                              f'{os.path.expanduser(p("token_file").value)})')
    try:
        uvicorn.run(app, host=host, port=port, log_level='info')
    finally:
        executor.shutdown()
        backend.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
