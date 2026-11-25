import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mecanum_forklift_web_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yinchheanyun',
    maintainer_email='yinchheanyun21@gmail.com',
    description='REST API, e-stop and dashboard for missions, status and video (no direct motion)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_api = mecanum_forklift_web_bridge.web_api_node:main',
            'estop_node = mecanum_forklift_web_bridge.estop_node:main',
        ],
    },
)
