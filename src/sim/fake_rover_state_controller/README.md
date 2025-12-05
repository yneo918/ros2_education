# fake_rover_state_controller

Simulated rover state controller with optional physics dynamics.

## Quick Start

```bash
# Basic usage
ros2 run fake_rover_state_controller sim_rover

# With parameters
ros2 run fake_rover_state_controller sim_rover --ros-args \
    -p robot_id:=rover \
    -p use_dynamics:=true \
    -p x:=1.0 -p y:=2.0 -p t:=0.0
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_id` | "p0" | Robot namespace |
| `x` | 0.0 | Initial X position [m] |
| `y` | 0.0 | Initial Y position [m] |
| `t` | 0.0 | Initial theta [rad] |
| `use_dynamics` | true | Enable physics model |
| `mass` | 5.0 | Robot mass [kg] |
| `response_time` | 0.3 | Velocity response time [s] |
| `direct_joy_control` | false | Subscribe to /joy/cmd_vel |

## Topics

**Subscribed:**
- `/{robot_id}/cmd_vel` (Twist) - Velocity commands
- `/joy/cmd_vel` (Twist) - Direct joystick (if enabled)

**Published:**
- `/{robot_id}/joint_states` (JointState) - For robot_state_publisher
- `/{robot_id}/pose2D` (Pose2D) - Current pose

## Simulation Modes

### Kinematics Mode (`use_dynamics:=false`)
- Instant velocity response
- No acceleration/deceleration
- Good for basic testing

### Dynamics Mode (`use_dynamics:=true`)
- Realistic physics with mass and inertia
- Gradual acceleration/deceleration
- Drag forces for natural stopping

---

# Reference Guide

## Physics Model

The dynamics simulation uses a first-order response model with drag:

### Linear Motion

```
F_command = m * (v_target - v_current) / T_response
F_drag = -c_drag * v_current
F_net = F_command + F_drag
a = clamp(F_net / m, -a_max, a_max)
v_new = v + a * dt
```

### Angular Motion

```
τ_command = I * (ω_target - ω_current) / T_response
τ_drag = -c_drag * ω_current
τ_net = τ_command + τ_drag
α = clamp(τ_net / I, -α_max, α_max)
ω_new = ω + α * dt
```

### Position Integration (Midpoint Method)

```
θ_mid = θ + 0.5 * ω * dt
θ_new = wrap_to_pi(θ + ω * dt)
x_new = x + v * cos(θ_mid) * dt
y_new = y + v * sin(θ_mid) * dt
```

## Physics Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `MASS_KG` | 5.0 | Robot mass |
| `INERTIA_KGM2` | 0.5 | Rotational inertia |
| `LINEAR_DRAG_COEFF` | 2.0 | Linear drag |
| `ANGULAR_DRAG_COEFF` | 1.0 | Angular drag |
| `MAX_FORCE_N` | 20.0 | Max propulsion force |
| `MAX_TORQUE_NM` | 5.0 | Max torque |
| `COMMAND_TIMEOUT_S` | 0.5 | Stop if no command |

**Derived:**
- `MAX_LINEAR_ACCEL = MAX_FORCE_N / MASS_KG = 4.0 m/s²`
- `MAX_ANGULAR_ACCEL = MAX_TORQUE_NM / INERTIA_KGM2 = 10.0 rad/s²`

## Coordinate System

```
        +Y
        ↑
        |
        |
+X ←----o  (θ = 0 points to +X)

θ positive = counter-clockwise
```

## ROS2 Concepts Demonstrated

### 1. Node Lifecycle
```python
rclpy.init(args=args)
node = MyNode()
rclpy.spin(node)       # Process callbacks
node.destroy_node()
rclpy.shutdown()
```

### 2. Parameter Declaration
```python
self.declare_parameters(
    namespace='',
    parameters=[
        ('param_name', default_value),
    ]
)
value = self.get_parameter('param_name').value
```

### 3. Publisher/Subscriber
```python
# Subscriber
self.create_subscription(MsgType, 'topic', callback, qos)

# Publisher
pub = self.create_publisher(MsgType, 'topic', qos)
pub.publish(msg)
```

### 4. Timer
```python
self.create_timer(period_seconds, callback)
```

## Code Structure

```
sim_rover.py
├── Constants (physics parameters)
├── _clamp() - Utility function
└── JointStates(Node)
    ├── __init__() - Setup params, pub/sub, timer
    ├── teleop_callback() - Handle velocity commands
    ├── on_timer() - Main simulation loop
    ├── _integrate_dynamics() - Physics model
    ├── _integrate_kinematics() - Simple model
    ├── _update_position() - Position integration
    ├── _wrap_to_pi() - Angle normalization
    └── _publish_states() - Publish JointState/Pose2D
```

## Behavior Comparison

| Aspect | Kinematics | Dynamics |
|--------|------------|----------|
| Response | Instant | Gradual |
| Stop behavior | Immediate | Coast then stop |
| Acceleration | Unlimited | Limited by force |
| Realism | Low | High |

## Tuning Guide

- **Faster response:** Decrease `response_time`
- **Heavier feel:** Increase `mass`
- **Quicker stopping:** Increase drag coefficients
- **More power:** Increase `MAX_FORCE_N` / `MAX_TORQUE_NM`
