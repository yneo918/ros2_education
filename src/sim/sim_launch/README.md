# sim_launch

Main launch package for the ROS2 Edu simulation.

## Quick Start

```bash
ros2 launch sim_launch simulation.launch.py
```

This single command launches the complete simulation environment.

## What Gets Launched

| Component | Package | Description |
|-----------|---------|-------------|
| robot_state_publisher | rover_description | Publishes TF from joint_states |
| static_transform_publisher | tf2_ros | World frame connection |
| sim_rover | fake_rover_state_controller | Simulates rover motion |
| rviz2 | rviz2 | 3D visualization |
| joy_node | joy | Joystick driver |
| joy2cmd | teleop_core | Joy to cmd_vel converter |
| virtual_joy | virtual_joy | GUI controller |

## Architecture

```
simulation.launch.py
├── rover.launch.py (rover_description)
│   ├── robot_state_publisher
│   ├── static_transform_publisher
│   └── sim_rover
├── display.launch.py (rover_description)
│   └── rviz2
├── joy.launch.py (teleop_core)
│   ├── joy_node
│   └── joy2cmd
└── virtual_joy (node)
```

---

# Reference Guide

## Launch File Composition

### IncludeLaunchDescription

Include other launch files:
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

launch_file = os.path.join(
    get_package_share_directory('package_name'),
    'launch', 'file.launch.py'
)

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(launch_file),
)
```

### Passing Arguments

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(launch_file),
    launch_arguments={
        'arg_name': 'value',
    }.items()
)
```

## Data Flow

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ virtual_joy │────▶│   joy2cmd   │────▶│  sim_rover  │
│  (or joy)   │ Joy │             │Twist│             │
└─────────────┘     └─────────────┘     └─────────────┘
                                              │
                                         JointState
                                              ▼
                    ┌─────────────────────────────────┐
                    │      robot_state_publisher      │
                    └─────────────────────────────────┘
                                   │
                                  TF
                                   ▼
                    ┌─────────────────────────────────┐
                    │             rviz2               │
                    └─────────────────────────────────┘
```

## Topic Overview

| Topic | Type | From | To |
|-------|------|------|----|
| `/joy` | Joy | virtual_joy/joy_node | joy2cmd |
| `/joy/cmd_vel` | Twist | joy2cmd | sim_rover |
| `/rover/joint_states` | JointState | sim_rover | robot_state_publisher |
| `/tf` | TFMessage | robot_state_publisher | rviz2 |

## Why Use a Meta-Launch Package?

1. **Single entry point**: One command starts everything
2. **Dependency management**: Changes propagate automatically
3. **Reusability**: Sub-launches work independently
4. **Clean separation**: Each package owns its launch logic

## Customization Examples

### Launch without virtual joystick (use real controller)
```bash
# Just don't launch virtual_joy separately
ros2 launch rover_description rover.launch.py &
ros2 launch rover_description display.launch.py &
ros2 launch teleop_core joy.launch.py
```

### Change robot starting position
Modify rover.launch.py or pass parameters:
```python
Node(
    package='fake_rover_state_controller',
    executable='sim_rover',
    parameters=[{
        'x': 5.0,
        'y': 3.0,
        't': 1.57,
    }]
)
```
