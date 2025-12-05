# ROS2 Education

An educational ROS2 simulation environment for learning robot development fundamentals.

## Quick Start

```bash
# Build
cd ros2_education
colcon build
source install/setup.bash

# Run simulation
ros2 launch sim_launch simulation.launch.py
```

**Controls:**
1. Hold **LB** button in virtual joystick window
2. Drag **Left Stick** up/down for forward/backward movement
3. Drag **Right Stick** left/right for rotation
4. Observe movement in RViz2

## What You'll Learn

- ROS2 node development (Python)
- Publisher/Subscriber patterns
- Launch file composition
- URDF/Xacro robot modeling
- TF (Transform) system
- Joystick teleoperation
- Physics simulation basics

## Requirements

- ROS2 Humble or later
- Python 3.10+
- PyQt6

---

# Repository Structure

```
ros2_education/
├── src/
│   ├── common/                         # Shared utilities
│   │   └── ros2_edu_util/              # PubSubManager, JoyBase, NavNode
│   ├── rover_base/                     # Robot base packages
│   │   └── teleop_core/                # Joy → cmd_vel conversion
│   └── sim/                            # Simulation packages
│       ├── sim_launch/                 # Main launch orchestrator
│       ├── rover_description/          # URDF/Xacro model + RViz config
│       ├── fake_rover_state_controller/# Physics simulation
│       └── virtual_joy/                # GUI joystick
└── README.md
```

## Package Overview

| Package | Type | Description |
|---------|------|-------------|
| **sim_launch** | Launch | Main entry point - launches everything |
| **rover_description** | CMake | Robot URDF/Xacro model and RViz config |
| **fake_rover_state_controller** | Python | Simulated rover with physics dynamics |
| **virtual_joy** | Python | PyQt6-based virtual Xbox controller |
| **teleop_core** | Python | Joystick to velocity command converter |
| **ros2_edu_util** | Python | Common utilities (PubSubManager, JoyBase) |

## Package Dependencies

```
sim_launch
├── rover_description
│   └── fake_rover_state_controller
│       └── ros2_edu_util
├── teleop_core
│   └── ros2_edu_util
└── virtual_joy
```

## Data Flow

```
┌─────────────┐      ┌─────────────┐      ┌─────────────┐
│ virtual_joy │─────▶│  teleop_core│─────▶│  sim_rover  │
│             │ /joy │  (joy2cmd)  │Twist │             │
└─────────────┘      └─────────────┘      └─────────────┘
                                                │
                                           JointState
                                                ▼
┌─────────────┐      ┌─────────────────────────────────┐
│    rviz2    │◀─────│     robot_state_publisher       │
│             │  TF  │                                 │
└─────────────┘      └─────────────────────────────────┘
```

## Topics

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/joy` | sensor_msgs/Joy | virtual_joy | joy2cmd |
| `/joy/cmd_vel` | geometry_msgs/Twist | joy2cmd | sim_rover |
| `/joy/enable` | std_msgs/Bool | joy2cmd | - |
| `/rover/joint_states` | sensor_msgs/JointState | sim_rover | robot_state_publisher |
| `/rover/pose2D` | geometry_msgs/Pose2D | sim_rover | - |
| `/tf` | tf2_msgs/TFMessage | robot_state_publisher | rviz2 |

## TF Frame Tree

```
world
└── rover/world
    └── x_axis
        └── y_axis
            └── theta_axis
                └── base_link
                    └── rover_link
```

---

# Reference Guide

## ROS2 Package Structure

### Python Package (ament_python)
```
package_name/
├── package.xml              # Dependencies and metadata
├── setup.py                 # Python build config
├── setup.cfg                # Entry point config
├── resource/package_name    # Ament marker file
├── package_name/            # Python module
│   ├── __init__.py
│   └── node.py
├── launch/                  # Launch files
└── config/                  # Config files (YAML)
```

### CMake Package (ament_cmake)
```
package_name/
├── package.xml
├── CMakeLists.txt
├── src/                     # Source files
├── include/                 # Headers
├── launch/
└── config/
```

## Node Development

### Basic Node Template
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Declare parameters
        self.declare_parameter('my_param', 'default')

        # Create pub/sub
        self.pub = self.create_publisher(MsgType, '/topic', 10)
        self.sub = self.create_subscription(
            MsgType, '/topic', self.callback, 10)

        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback(self, msg):
        pass

    def timer_callback(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Using PubSubManager (ros2_edu_util)
```python
from ros2_edu_util.my_ros_module import PubSubManager

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.pubsub = PubSubManager(self)

        self.pubsub.create_publisher(Twist, '/cmd_vel', 10)
        self.pubsub.create_subscription(Joy, '/joy', self.callback, 10)

    def callback(self, msg):
        twist = Twist()
        self.pubsub.publish('/cmd_vel', twist)
```

## Launch Files

### Basic Launch
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            parameters=[{'param': 'value'}],
            output='screen',
        ),
    ])
```

### Include Other Launch Files
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('other_package')
    launch_file = os.path.join(pkg_dir, 'launch', 'other.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'arg': 'value'}.items()
        ),
    ])
```

## URDF/Xacro

### Joint Types
| Type | Description | Example |
|------|-------------|---------|
| `fixed` | No movement | base_link to sensor |
| `revolute` | Rotation with limits | wheel, arm joint |
| `continuous` | Unlimited rotation | wheel |
| `prismatic` | Linear translation | elevator, X/Y axis |

### Xacro Features
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">
  <!-- Arguments -->
  <xacro:arg name="color" default="1.0 0.0 0.0"/>

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.05"/>

  <!-- Macros -->
  <xacro:macro name="wheel" params="name x y">
    <link name="${name}"/>
    <joint name="${name}_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${name}"/>
      <origin xyz="${x} ${y} 0"/>
    </joint>
  </xacro:macro>

  <!-- Usage -->
  <xacro:wheel name="left" x="0" y="0.1"/>
</robot>
```

## Physics Simulation

This repository's sim_rover demonstrates a simple physics model:

### Dynamics Equations
```
Force:   F = m * (v_target - v_current) / T_response - drag * v
Accel:   a = clamp(F / m, -a_max, a_max)
Velocity: v_new = v + a * dt
Position: x_new = x + v * cos(θ) * dt
```

### Parameters
| Parameter | Value | Effect |
|-----------|-------|--------|
| mass | 5.0 kg | Higher = slower acceleration |
| response_time | 0.3 s | Lower = quicker response |
| drag_coeff | 2.0 | Higher = quicker stopping |

## Common Commands

```bash
# Build
colcon build
colcon build --packages-select pkg_name

# Run
ros2 run package_name executable
ros2 launch package_name file.launch.py

# Topics
ros2 topic list
ros2 topic echo /topic
ros2 topic hz /topic
ros2 topic info /topic

# Nodes
ros2 node list
ros2 node info /node_name

# Parameters
ros2 param list /node_name
ros2 param get /node_name param
ros2 param set /node_name param value

# TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo frame1 frame2

# Debug
rqt
ros2 run rqt_console rqt_console
ros2 run rqt_graph rqt_graph
```

## Best Practices

1. **One responsibility per node** - Keep nodes focused
2. **Use parameters** for configurable values
3. **Handle shutdown gracefully** with try/finally
4. **Follow naming conventions** - snake_case for topics
5. **Document with docstrings** - What, not how
6. **Use launch files** for multi-node setups
7. **Separate packages by function** - sim, base, common

## Learning Path

1. **Start here**: Run simulation, understand data flow
2. **Explore teleop_core**: Simple Joy → Twist conversion
3. **Study sim_rover**: Physics simulation, timer-based updates
4. **Examine rover_description**: URDF/Xacro, robot_state_publisher
5. **Review virtual_joy**: PyQt6 + ROS2 integration
6. **Create your own**: New node using ros2_edu_util

## Package READMEs

Each package has its own README with:
- **Top section**: Quick start, parameters, topics
- **Bottom section**: Reference guide with detailed explanations

| Package | Key Learning Points |
|---------|---------------------|
| [fake_rover_state_controller](src/sim/fake_rover_state_controller/README.md) | Physics dynamics, timer callbacks |
| [ros2_edu_util](src/common/ros2_edu_util/README.md) | PubSubManager pattern, JoyBase |
| [teleop_core](src/rover_base/teleop_core/README.md) | Joy processing, REP-103 coordinates |
| [virtual_joy](src/sim/virtual_joy/README.md) | PyQt6 + ROS2, signal handling |
| [rover_description](src/sim/rover_description/README.md) | URDF/Xacro, TF frames |
| [sim_launch](src/sim/sim_launch/README.md) | Launch composition, architecture |

---

## License

ECL-2.0
