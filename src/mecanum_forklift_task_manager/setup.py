import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mecanum_forklift_task_manager'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yinchheanyun',
    maintainer_email='yinchheanyun21@gmail.com',
    description='Mission manager: pick, carry, drop and return with timeouts, retries and cancel',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = mecanum_forklift_task_manager.mission_node:main',
        ],
    },
)
