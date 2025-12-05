# rover_description

Robot URDF/Xacro model and RViz configuration for the rover.

## Quick Start

```bash
# Launch robot model + RViz
ros2 launch rover_description rover.launch.py
ros2 launch rover_description display.launch.py
```

## Contents

```
rover_description/
├── launch/
│   ├── rover.launch.py    # Robot state publisher + sim_rover
│   └── display.launch.py  # RViz2 visualization
├── src/description/
│   └── rover_robot.xacro  # Robot model
├── meshes/
│   └── rover_cad.stl      # 3D mesh
└── rviz/
    └── rover.rviz         # RViz config
```

## Launch Files

### rover.launch.py
Launches robot_state_publisher and sim_rover.

**Parameters:**
- `use_sim_time`: Enable simulation time (default: True)

### display.launch.py
Launches RViz2 with preconfigured view.

**Parameters:**
- `rvizconfig`: Path to RViz config file
- `use_sim_time`: Enable simulation time (default: True)

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/rover/robot_description` | String | URDF string |
| `/rover/joint_states` | JointState | Joint positions (subscribed) |

---

# Reference Guide

## TF Frame Tree

```
world
└── rover/world (static)
    └── x_axis (prismatic: X translation)
        └── y_axis (prismatic: Y translation)
            └── theta_axis (revolute: rotation)
                └── base_link (fixed)
                    └── rover_link (fixed, visual)
```

## Xacro Model Structure

### Arguments
| Argument | Default | Description |
|----------|---------|-------------|
| `r` | 1.0 | Red color component |
| `g` | 1.0 | Green color component |
| `b` | 1.0 | Blue color component |
| `a` | 1.0 | Alpha (transparency) |
| `mesh_enabled` | true | Show STL mesh |

### Joints
| Joint | Type | Axis | Description |
|-------|------|------|-------------|
| `w_to_x` | prismatic | X | World to X position |
| `x_to_y` | prismatic | Y | X to Y position |
| `y_to_t` | revolute | Z | Y to theta rotation |
| `t_to_b` | fixed | - | Theta to base_link |
| `base_joint` | fixed | - | Base to visual mesh |

### Why Prismatic Joints?

This model uses prismatic joints (X, Y) + revolute (theta) to simulate 2D ground robot motion. Unlike wheel-based models:
- No need to calculate wheel odometry
- Direct control of position via joint_states
- Simple integration with sim_rover node

## URDF vs Xacro

**URDF** (Unified Robot Description Format):
- XML format for robot structure
- Static, no variables or macros

**Xacro** (XML Macro):
- Extends URDF with macros and parameters
- Processed at build/launch time via `xacro` command

```xml
<!-- Xacro argument definition -->
<xacro:arg name="r" default="1.0"/>

<!-- Using argument -->
<color rgba="${r} ${g} ${b} ${a}"/>

<!-- Macro definition -->
<xacro:macro name="wheel" params="name x y">
  <link name="${name}"/>
</xacro:macro>

<!-- Macro usage -->
<xacro:wheel name="left_wheel" x="0" y="0.1"/>
```

## Robot State Publisher

Converts JointState messages to TF transforms:

```
joint_states → robot_state_publisher → TF
```

Configuration in launch file:
```python
Node(
    package="robot_state_publisher",
    parameters=[{
        "robot_description": Command(["xacro ", xacro_file]),
        "frame_prefix": "rover/",  # Optional namespace
    }]
)
```

## STL Mesh Integration

```xml
<visual>
  <geometry>
    <mesh filename="package://rover_description/meshes/rover_cad.stl"
          scale="0.001 0.001 0.001"/>
  </geometry>
  <material name="Color">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
</visual>
```

- `filename`: Package-relative path using `package://`
- `scale`: Convert mesh units (mm to m: 0.001)
- `material`: Color applied to mesh
