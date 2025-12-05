"""Launch RViz2 with rover visualization configuration."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')

    # RViz config file path
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rover.rviz')

    return LaunchDescription([
        # Specify RViz config file path
        DeclareLaunchArgument("rvizconfig", default_value=default_rviz_config_path, description="Absolute path to rviz config file"),

        # Enable time for Gazebo and simulation
        DeclareLaunchArgument("use_sim_time", default_value="True", description="Flag to enable use_sim_time"),

        # Launch RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
    ])
