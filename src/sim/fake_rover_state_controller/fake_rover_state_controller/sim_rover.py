"""
Simulated Rover State Controller with optional physics dynamics.

This node simulates rover motion and publishes joint states for visualization.
Supports both simple kinematics (instant response) and physics-based dynamics.
"""

import rclpy
from rclpy.node import Node

import time
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D

from ros2_edu_util.my_ros_module import PubSubManager

# Simulation
UPDATE_RATE_HZ = 10.0

# Velocity limits
MAX_TRANS = 1.0       # [m/s]
MAX_ROTATE = 1.0      # [rad/s]

# Physics parameters
MASS_KG = 5.0
INERTIA_KGM2 = 0.5
RESPONSE_TIME_S = 0.3
ANG_RESPONSE_TIME_S = 0.3
LINEAR_DRAG_COEFF = 2.0
ANGULAR_DRAG_COEFF = 1.0
MAX_FORCE_N = 20.0
MAX_TORQUE_NM = 5.0
COMMAND_TIMEOUT_S = 0.5

# Derived limits: a_max = F_max / m
MAX_LINEAR_ACCEL = MAX_FORCE_N / MASS_KG
MAX_ANGULAR_ACCEL = MAX_TORQUE_NM / INERTIA_KGM2


def _clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))


class JointStates(Node):
    """Simulates rover motion and publishes JointState/Pose2D."""

    def __init__(self):
        super().__init__('rover_sim')
        self.pubsub = PubSubManager(self)

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', "p0"),
                ('x', 0.0),
                ('y', 0.0),
                ('t', 0.0),
                ('direct_joy_control', False),
                ('use_dynamics', True),
                ('mass', MASS_KG),
                ('response_time', RESPONSE_TIME_S),
            ]
        )

        # Log parameters
        for name, param in self._parameters.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")

        # Load parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.use_dynamics = self.get_parameter('use_dynamics').value
        self.mass = self.get_parameter('mass').value
        self.response_time = self.get_parameter('response_time').value

        # Initialize state
        self.position = {
            'x': float(self.get_parameter('x').value),
            'y': float(self.get_parameter('y').value),
            'theta': float(self.get_parameter('t').value)
        }
        self.joint_names = ['w_to_x', 'x_to_y', 'y_to_t']
        self.current_velocity = {'linear': 0.0, 'angular': 0.0}
        self.target_velocity = {'linear': 0.0, 'angular': 0.0}
        self.last_command_time = self.get_clock().now()
        self.last_update_time = None

        # Setup pub/sub
        self.pubsub.create_subscription(
            Twist, f'/{self.robot_id}/cmd_vel', self.teleop_callback, 10)
        self.pubsub.create_publisher(
            JointState, f'/{self.robot_id}/joint_states', 10)
        self.pubsub.create_publisher(
            Pose2D, f'/{self.robot_id}/pose2D', 10)

        if self.get_parameter('direct_joy_control').value:
            self.get_logger().info("Direct joy control enabled.")
            self.pubsub.create_subscription(
                Twist, '/joy/cmd_vel', self.teleop_callback, 10)

        mode = "dynamics" if self.use_dynamics else "kinematics"
        self.get_logger().info(f"Simulation mode: {mode}")

        # Start timer
        self.timer = self.create_timer(1.0 / UPDATE_RATE_HZ, self.on_timer)

    def teleop_callback(self, msg: Twist) -> None:
        """Store velocity command and update timestamp."""
        self.target_velocity['linear'] = _clamp(msg.linear.x, -MAX_TRANS, MAX_TRANS)
        self.target_velocity['angular'] = _clamp(msg.angular.z, -MAX_ROTATE, MAX_ROTATE)
        self.last_command_time = self.get_clock().now()

    def on_timer(self) -> None:
        """Main loop: calculate dt, update state, publish."""
        now = self.get_clock().now()

        # Calculate dt
        if self.last_update_time is None:
            dt = 1.0 / UPDATE_RATE_HZ
        else:
            dt = (now - self.last_update_time).nanoseconds / 1e9
            if dt <= 0.0:
                dt = 1.0 / UPDATE_RATE_HZ
        self.last_update_time = now

        # Command timeout
        if (now - self.last_command_time).nanoseconds / 1e9 > COMMAND_TIMEOUT_S:
            self.target_velocity['linear'] = 0.0
            self.target_velocity['angular'] = 0.0

        # Update position
        if self.use_dynamics:
            self._integrate_dynamics(dt)
        else:
            self._integrate_kinematics(dt)

        self._publish_states()

    def _integrate_dynamics(self, dt: float) -> None:
        """Physics-based update with mass, drag, and acceleration limits."""
        # Linear: F = ma, with drag and response time control
        linear_error = self.target_velocity['linear'] - self.current_velocity['linear']
        force = self.mass * linear_error / self.response_time - LINEAR_DRAG_COEFF * self.current_velocity['linear']
        accel = _clamp(force / self.mass, -MAX_LINEAR_ACCEL, MAX_LINEAR_ACCEL)
        self.current_velocity['linear'] = _clamp(
            self.current_velocity['linear'] + accel * dt, -MAX_TRANS, MAX_TRANS)

        # Angular: τ = Iα, with drag
        angular_error = self.target_velocity['angular'] - self.current_velocity['angular']
        torque = INERTIA_KGM2 * angular_error / ANG_RESPONSE_TIME_S - ANGULAR_DRAG_COEFF * self.current_velocity['angular']
        ang_accel = _clamp(torque / INERTIA_KGM2, -MAX_ANGULAR_ACCEL, MAX_ANGULAR_ACCEL)
        self.current_velocity['angular'] = _clamp(
            self.current_velocity['angular'] + ang_accel * dt, -MAX_ROTATE, MAX_ROTATE)

        self._update_position(dt)

    def _integrate_kinematics(self, dt: float) -> None:
        """Simple kinematics: instant velocity response."""
        self.current_velocity['linear'] = self.target_velocity['linear']
        self.current_velocity['angular'] = self.target_velocity['angular']
        self._update_position(dt)

    def _update_position(self, dt: float) -> None:
        """Integrate velocity to position using midpoint method."""
        theta_mid = self.position['theta'] + 0.5 * self.current_velocity['angular'] * dt
        self.position['theta'] = self._wrap_to_pi(
            self.position['theta'] + self.current_velocity['angular'] * dt)
        self.position['x'] += self.current_velocity['linear'] * math.cos(theta_mid) * dt
        self.position['y'] += self.current_velocity['linear'] * math.sin(theta_mid) * dt

    def _wrap_to_pi(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _publish_states(self) -> None:
        """Publish JointState and Pose2D messages."""
        t = time.time()

        # JointState
        js = JointState()
        js.header.stamp.sec = int(t // 1)
        js.header.stamp.nanosec = int((t % 1) * 1e9)
        js.name = self.joint_names
        js.position = [self.position['x'], self.position['y'], self.position['theta']]
        self.pubsub.publish(f'/{self.robot_id}/joint_states', js)

        # Pose2D
        pose = Pose2D()
        pose.x = self.position['x']
        pose.y = self.position['y']
        pose.theta = self.position['theta']
        self.pubsub.publish(f'/{self.robot_id}/pose2D', pose)


def main(args=None):
    rclpy.init(args=args)
    node = JointStates()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
