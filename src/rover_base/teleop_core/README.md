# teleop_core

Joystick teleoperation package for rover control.

## Overview

This package converts joystick input (sensor_msgs/Joy) to velocity commands (geometry_msgs/Twist) for robot control.

## Nodes

### joy2cmd

Converts joystick messages to cmd_vel.

```bash
ros2 run teleop_core joy2cmd
```

**Parameters:**
- `lx`: Axis name for linear X (default: "LY")
- `ly`: Axis name for linear Y (default: "LX")
- `az`: Axis name for angular Z (default: "RX")
- `en`: Button name for enable (default: "LB")

## Launch Files

### joy.launch.py

Launches both the joy_node (joystick driver) and joy2cmd.

```bash
ros2 launch teleop_core joy.launch.py
```

## Topics

### Subscribed
- `/joy` (sensor_msgs/Joy) - Raw joystick input

### Published
- `/joy/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/joy/enable` (std_msgs/Bool) - Enable state

## Control Mapping

Default Xbox controller mapping:

| Control | Action |
|---------|--------|
| LB (hold) | Enable movement |
| Left Stick Y | Forward/Backward |
| Left Stick X | Strafe (if supported) |
| Right Stick X | Rotation |

### Why LY maps to linear.x?

This mapping follows [REP-103](https://www.ros.org/reps/rep-0103.html) (Standard Units of Measure and Coordinate Conventions):

| REP-103 Axis | Direction | Joystick | Human Intuition |
|--------------|-----------|----------|-----------------|
| X | Forward | LY (up/down) | Push up = go forward |
| Y | Left | LX (left/right) | Push left = go left |

The robot's X-axis points forward, but humans intuitively push the stick **up** to move forward. Therefore:
- `lx` (robot linear.x) ← `LY` (stick vertical)
- `ly` (robot linear.y) ← `LX` (stick horizontal)

This ensures intuitive control while maintaining REP-103 compliance.

## Velocity Limits

- Max linear velocity: 1.0 m/s
- Max angular velocity: 1.0 rad/s

## Architecture

```
joy_node (ROS2 joy package)
    │
    ▼ /joy
joy2cmd (teleop_core)
    │
    ▼ /joy/cmd_vel
sim_rover (fake_rover_state_controller)
```

## Code Structure

```python
class JoyCmd(JoyBase):
    def __init__(self):
        # Setup parameters and publishers

    def joy_callback(self, msg):
        # Process joystick input

    def _process_movement(self, msg):
        # Convert joy to cmd_vel when enabled
```

## Learning Points

This package demonstrates:
- Processing sensor_msgs/Joy messages
- Parameter-based control mapping
- Dead-man switch (enable button) pattern
- Velocity scaling and limiting
- Inheriting from utility base classes

## Dependencies

- joy (ROS2 package for joystick drivers)
- ros2_edu_util (JoyBase class)
