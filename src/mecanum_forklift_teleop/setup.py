import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mecanum_forklift_teleop'

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
    description='Keyboard and PS4 teleoperation for the mecanum forklift AMR',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_node = mecanum_forklift_teleop.keyboard_node:main',
            'ps4_node = mecanum_forklift_teleop.ps4_node:main',
        ],
    },
)
