"""Launch robot state publisher and simulated rover controller."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
import launch_ros
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')
    xacro_file = os.path.join(pkg_share, f'src/description/rover_robot.xacro')

    main_nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="state_publisher",
            namespace="rover",
            output="screen",
            parameters=[{
                "robot_description": Command([
                    "xacro ", 
                    xacro_file, 
                    f" r:=1.0 g:=0.0 b:=0.0 a:=1.0"
                ]),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "frame_prefix": f"rover/",
            }]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"rover/world"],
        ),
        Node(
            package='fake_rover_state_controller',
            executable='sim_rover',
            name='sim_rover',
            output='screen',
            parameters=[{
                'robot_id': "rover",
                'x': 0.0,
                'y': 0.0,
                't': 0.0,
                'direct_joy_control': True
            }]
        ),
    ]
    return main_nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="True", description="Flag to enable use_sim_time"),

        OpaqueFunction(function=launch_setup),
    ])
