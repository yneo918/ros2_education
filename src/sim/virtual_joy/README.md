# virtual_joy

GUI-based virtual Xbox controller for ROS2.

## Overview

This package provides a PyQt6-based GUI that emulates an Xbox controller, publishing sensor_msgs/Joy messages. Useful for testing teleoperation without physical hardware.

## Node

### virtual_joy

Launches a GUI window with virtual joystick controls.

```bash
ros2 run virtual_joy virtual_joy
```

## GUI Controls

### Analog Sticks
- **Left/Right Stick**: 2D draggable circular area
  - Drag the knob to move the stick
  - **Sticky behavior**: Position is fixed when mouse is released (does not return to center)
  - **RST button**: Reset individual stick to center (left and right separate)
- **Axis convention** (matches real Xbox controller):
  - X-axis: Right = +1, Left = -1
  - Y-axis: Up = +1, Down = -1

### Triggers
- **LT/RT**: Horizontal progress bar (drag to adjust 0-100%)

### Bumpers
- **LB/RB**: Click to toggle

### Face Buttons
- **A** (green), **B** (red), **X** (blue), **Y** (yellow): Diamond layout, click to toggle

### D-Pad
- **Up/Down/Left/Right**: Cross layout, click to toggle

### Center Buttons
- **Back**, **Xbox**, **Start**: Click to toggle

### Stick Buttons
- **L3/R3**: Click to toggle (stick press)

## Topics

### Published
- `/joy` (sensor_msgs/Joy) - Joystick state at 10Hz

## Joy Message Mapping

### Axes (index)
| Index | Control |
|-------|---------|
| 0 | Left X |
| 1 | Left Y |
| 2 | LT |
| 3 | Right X |
| 4 | Right Y |
| 5 | RT |
| 6 | D-Pad Y |
| 7 | D-Pad X |

### Buttons (index)
| Index | Button |
|-------|--------|
| 0 | A |
| 1 | B |
| 2 | X |
| 3 | Y |
| 4 | LB |
| 5 | RB |
| 6 | Back |
| 7 | Start |
| 8 | Xbox |
| 9 | L Stick |
| 10 | R Stick |

## Usage with teleop_core

The virtual joystick works with teleop_core's joy2cmd node:
1. Hold **LB** button
2. Drag **Left Stick** up/down for forward/backward
3. Drag **Right Stick** left/right for rotation

## Implementation Notes

### QPainter-based Rendering

The GUI is rendered using QPainter for a realistic Xbox controller appearance:
- Dark theme with gradient controller body
- Color-coded ABXY buttons
- Visual feedback for active states

### Sticky Stick Behavior

Unlike real controllers, sticks do not auto-center on release. This allows:
- Maintaining constant velocity without holding the mouse
- Simultaneous operation of both sticks
- Individual reset via RST buttons

### Signal Handling

The node properly handles Ctrl+C termination:

```python
# SIGINT handler for clean shutdown
signal.signal(signal.SIGINT, lambda *args: app.quit())

# Timer to allow Python signal processing in Qt event loop
timer = QTimer()
timer.timeout.connect(lambda: None)
timer.start(100)
```

## Learning Points

This package demonstrates:
- Integrating PyQt6 with ROS2
- Custom widget rendering with QPainter
- Publishing sensor_msgs/Joy messages
- Handling Unix signals in Qt applications
- Creating interactive ROS2 tools

## Dependencies

- PyQt6
- rclpy
- sensor_msgs
