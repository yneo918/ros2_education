"""Simple joystick command node without GUI."""

import rclpy
from std_msgs.msg import Bool, Int16, String, Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from typing import Dict, Optional, Tuple, Any

from ros2_edu_util.my_ros_module import JoyBase

DEFAULT_QOS = 10
MAX_VEL_TRANS = 1.0  # m/s
MAX_VEL_ROT = 1.0    # rad/s

class JoyCmd(JoyBase):
    """Base class for joystick command processing nodes."""
    
    def __init__(self, node_name: str = 'joy_cmd'):
        super().__init__(node_name)
        
        # Control mappings (will be set by derived classes)
        self.lx_axis: Optional[int] = None
        self.ly_axis: Optional[int] = None
        self.az_axis: Optional[int] = None
        self.en_button: Optional[int] = None
        
        self._setup_parameters()
        self._setup_publishers()
        
    def _setup_parameters(self) -> None:
        """Declare and load ROS parameters."""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lx', "LY"),
                ('ly', "LX"),
                ('az', "RX"),
                ('en', "LB"),
            ]
        )
        
        # Load control mappings
        self.lx_axis = self.axis_dict.get(self.get_parameter('lx').value)
        self.ly_axis = self.axis_dict.get(self.get_parameter('ly').value)
        self.az_axis = self.axis_dict.get(self.get_parameter('az').value)
        self.en_button = self.button_dict.get(self.get_parameter('en').value)
        
        self._log_parameters()
        
    def _log_parameters(self) -> None:
        """Log initialization parameters."""
        self.get_logger().info(
            f"JoyCmd Node Initialized\n"
            f"lx: {self.get_parameter('lx').value}\n"
            f"ly: {self.get_parameter('ly').value}\n"
            f"az: {self.get_parameter('az').value}\n"
            f"en: {self.get_parameter('en').value}\n"
        )
        
    def _setup_publishers(self) -> None:
        """Setup ROS publishers."""
        self.pubsub.create_publisher(Twist, '/joy/cmd_vel', DEFAULT_QOS)
        self.pubsub.create_publisher(Bool, '/joy/enable', DEFAULT_QOS)
       
    def joy_callback(self, msg: Joy) -> None:
        """Process joystick input messages."""
        try:
            self._process_movement(msg)
        except Exception as e:
            self.get_logger().error(f"Error in joy_callback: {e}")
        
    def _process_movement(self, msg: Joy) -> None:
        """Process joystick movement commands."""
        if msg.buttons[self.en_button] == 1:
            # Create velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = msg.axes[self.lx_axis] * MAX_VEL_TRANS
            if self.ly_axis is not None:
                cmd_vel.linear.y = msg.axes[self.ly_axis] * MAX_VEL_TRANS
            cmd_vel.angular.z = -msg.axes[self.az_axis] * MAX_VEL_ROT
            
            # Publish velocity and enable
            self.pubsub.publish('/joy/cmd_vel', cmd_vel)
            self.pubsub.publish('/joy/enable', Bool(data=True))
        else:
            # Stop movement
            self.pubsub.publish('/joy/cmd_vel', Twist())
            self.pubsub.publish('/joy/enable', Bool(data=False))

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        joy_cmd = JoyCmd()
        rclpy.spin(joy_cmd)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'joy_cmd' in locals():
            joy_cmd.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()