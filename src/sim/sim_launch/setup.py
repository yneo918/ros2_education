"""Setup configuration for sim_launch package."""

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sim_launch'

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
    description='Main simulation launch orchestrating all components',
    license='ECL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
