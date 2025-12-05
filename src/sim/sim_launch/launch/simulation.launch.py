"""
Main simulation launch file.

Launches the complete simulation environment:
- Robot model and state publisher
- RViz2 visualization
- Joystick driver and teleop
- Virtual joystick GUI
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'rover_description'
    rover_launch_file = os.path.join(get_package_share_directory(pkg_name), 'launch', 'rover.launch.py')
    display_launch_file = os.path.join(get_package_share_directory(pkg_name), 'launch', 'display.launch.py')
    pkg_name = 'teleop_core'
    teleop_launch_file = os.path.join(get_package_share_directory(pkg_name), 'joy.launch.py')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rover_launch_file),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch_file),
        ),
        Node(
            package='virtual_joy',
            executable='virtual_joy',
        )
    ])
