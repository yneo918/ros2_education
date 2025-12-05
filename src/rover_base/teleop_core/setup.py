"""Setup configuration for teleop_core package."""

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teleop_core'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS2 Education Project',
    maintainer_email='ros2edu@example.com',
    description='Joystick teleoperation: converts Joy to cmd_vel',
    license='ECL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy2cmd = teleop_core.run_joy:main',
        ],
    },
)
