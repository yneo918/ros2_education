"""
ROS2 utility classes for common patterns.

Provides:
- PubSubManager: Centralized publisher/subscriber management
- JoyBase: Base class for joystick nodes with Xbox controller mapping
- NavNode: Navigation utilities with GPS calculations
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32MultiArray
from sensor_msgs.msg import NavSatFix

from math import sin, cos, asin, atan2, sqrt, degrees, pi, radians
from typing import Dict, List, Any, Callable


class PubSubManager:
    """Centralized manager for ROS2 publishers and subscribers."""

    def __init__(self, node: Node):
        if not isinstance(node, Node):
            raise TypeError("node must be an instance of rclpy.node.Node")
        self.node = node
        self._subscribers: Dict[str, Any] = {}
        self._publishers: Dict[str, Any] = {}

    def create_subscription(self, msg_type: type, topic_name: str,
                            callback: Callable, qos: int = 10, **kwargs) -> None:
        """Create and register a subscriber."""
        if topic_name in self._subscribers:
            raise ValueError(f"Subscriber for '{topic_name}' already exists")
        subscriber = self.node.create_subscription(
            msg_type, topic_name, callback, qos, **kwargs)
        self._subscribers[topic_name] = subscriber
        self.node.get_logger().info(f"Created subscriber: {topic_name}")

    def create_publisher(self, msg_type: type, topic_name: str,
                         qos: int = 10, **kwargs) -> None:
        """Create and register a publisher."""
        if topic_name in self._publishers:
            raise ValueError(f"Publisher for '{topic_name}' already exists")
        publisher = self.node.create_publisher(msg_type, topic_name, qos, **kwargs)
        self._publishers[topic_name] = publisher
        self.node.get_logger().info(f"Created publisher: {topic_name}")

    def publish(self, topic_name: str, msg: Any) -> None:
        """Publish message to registered topic."""
        if topic_name not in self._publishers:
            raise ValueError(f"Publisher for '{topic_name}' not found")
        self._publishers[topic_name].publish(msg)

    def has_subscriber(self, topic_name: str) -> bool:
        return topic_name in self._subscribers

    def has_publisher(self, topic_name: str) -> bool:
        return topic_name in self._publishers

    def get_subscriber_topics(self) -> List[str]:
        return list(self._subscribers.keys())

    def get_publisher_topics(self) -> List[str]:
        return list(self._publishers.keys())


class JoyBase(Node):
    """Base class for joystick-based nodes with Xbox controller mapping."""

    # Xbox controller button mapping
    BUTTON_MAP = {
        "A": 0, "B": 1, "X": 2, "Y": 3,
        "LB": 4, "RB": 5,
        "BACK": 6, "START": 7, "POWER": 8,
        "LS": 9, "RS": 10
    }

    # Xbox controller axis mapping
    AXIS_MAP = {
        "LX": 0, "LY": 1, "LT": 2,
        "RX": 3, "RY": 4, "RT": 5,
        "cross_lr": 6, "cross_ud": 7
    }

    def __init__(self, node_name='joy_node'):
        super().__init__(node_name)
        self.button_dict = self.BUTTON_MAP.copy()
        self.axis_dict = self.AXIS_MAP.copy()
        self.pubsub = PubSubManager(self)
        self.pubsub.create_subscription(Joy, 'joy', self.joy_callback)
        self.prev_joy = Joy()

    def joy_callback(self, msg):
        """Override this method to handle joystick input."""
        self.joy_toggle(msg)

    def joy_toggle(self, joy_msg) -> List[int]:
        """Detect button state changes (press/release)."""
        if len(self.prev_joy.buttons) != len(joy_msg.buttons):
            self.prev_joy = joy_msg
            return [0] * len(joy_msg.buttons)

        ret = [self.prev_joy.buttons[i] - joy_msg.buttons[i]
               for i in range(len(joy_msg.buttons))]
        self.prev_joy = joy_msg
        return ret


class NavNode(Node):
    """Navigation node with GPS utility functions."""

    R = 6373.0  # Earth radius [km]

    def __init__(self, node_name='nav_node'):
        super().__init__(node_name)
        self.pubsub = PubSubManager(self)

    def get_bearing_distance(self, lat0, lon0, lat1, lon1):
        """Calculate bearing and distance between two GPS coordinates."""
        delta_lat = radians(lat1) - radians(lat0)
        delta_lon = radians(lon1) - radians(lon0)
        cos_lat1 = cos(radians(lat1))
        cos_lat0 = cos(radians(lat0))

        # Bearing
        bearing_x = cos_lat1 * sin(delta_lon)
        bearing_y = cos_lat0 * sin(radians(lat1)) - sin(radians(lat0)) * cos_lat1 * cos(delta_lon)
        yaw = -atan2(bearing_x, bearing_y)
        if yaw < 0:
            yaw += 2 * pi
        bearing = degrees(yaw)

        # Distance (Haversine formula)
        a = sin(delta_lat / 2) ** 2 + cos_lat1 * cos_lat0 * sin(delta_lon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        dist = self.R * c * 1000  # [m]

        return bearing, dist

    def get_target_pos(self, lat0, lon0, bearing, dist):
        """Calculate target GPS position from bearing and distance."""
        lat0 = radians(lat0)
        lon0 = radians(lon0)
        bearing = radians(bearing)
        d = (dist / 1000) / self.R

        lat1 = asin(sin(lat0) * cos(d) + cos(lat0) * sin(d) * cos(bearing))
        lon1 = lon0 + atan2(sin(bearing) * sin(d) * cos(lat0),
                           cos(d) - sin(lat0) * sin(lat1))
        return degrees(lat1), degrees(lon1)


def main(args=None):
    rclpy.init(args=args)
    node = JoyBase()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
