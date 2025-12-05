# ros2_edu_util

Common ROS2 utility classes for the ros2_education project.

## Classes

### PubSubManager
Centralized publisher/subscriber management.

```python
from ros2_edu_util.my_ros_module import PubSubManager

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.pubsub = PubSubManager(self)

        self.pubsub.create_publisher(String, '/output', 10)
        self.pubsub.create_subscription(String, '/input', self.callback, 10)

    def callback(self, msg):
        self.pubsub.publish('/output', String(data='response'))
```

### JoyBase
Base class for joystick nodes with Xbox controller mapping.

```python
from ros2_edu_util.my_ros_module import JoyBase

class MyJoyNode(JoyBase):
    def __init__(self):
        super().__init__('my_joy')

    def joy_callback(self, msg):
        if msg.buttons[self.button_dict['A']]:
            # A button pressed
            pass
```

### NavNode
Navigation utilities with GPS calculations.

```python
from ros2_edu_util.my_ros_module import NavNode

class MyNavNode(NavNode):
    def __init__(self):
        super().__init__('my_nav')
        bearing, dist = self.get_bearing_distance(lat0, lon0, lat1, lon1)
```

---

# Reference Guide

## PubSubManager API

| Method | Description |
|--------|-------------|
| `create_subscription(msg_type, topic, callback, qos)` | Register subscriber |
| `create_publisher(msg_type, topic, qos)` | Register publisher |
| `publish(topic, msg)` | Publish to topic |
| `has_subscriber(topic)` | Check if subscriber exists |
| `has_publisher(topic)` | Check if publisher exists |
| `get_subscriber_topics()` | List subscriber topics |
| `get_publisher_topics()` | List publisher topics |

## Xbox Controller Mapping

### Buttons (JoyBase.BUTTON_MAP)
| Name | Index | Name | Index |
|------|-------|------|-------|
| A | 0 | RB | 5 |
| B | 1 | BACK | 6 |
| X | 2 | START | 7 |
| Y | 3 | POWER | 8 |
| LB | 4 | LS | 9 |
| | | RS | 10 |

### Axes (JoyBase.AXIS_MAP)
| Name | Index | Description |
|------|-------|-------------|
| LX | 0 | Left stick X |
| LY | 1 | Left stick Y |
| LT | 2 | Left trigger |
| RX | 3 | Right stick X |
| RY | 4 | Right stick Y |
| RT | 5 | Right trigger |
| cross_lr | 6 | D-pad left/right |
| cross_ud | 7 | D-pad up/down |

## NavNode GPS Functions

### get_bearing_distance(lat0, lon0, lat1, lon1)
Calculate bearing [degrees] and distance [meters] between two coordinates.

Uses Haversine formula:
```
a = sin²(Δlat/2) + cos(lat0) * cos(lat1) * sin²(Δlon/2)
c = 2 * atan2(√a, √(1-a))
distance = R * c
```

### get_target_pos(lat0, lon0, bearing, dist)
Calculate target GPS position from start point, bearing, and distance.

## Design Pattern: PubSubManager

**Problem:** Publishers scattered across node code, hard to track topics.

**Solution:** Centralize in PubSubManager with topic-name-based access.

```python
# Without PubSubManager
self.pub1 = self.create_publisher(...)
self.pub2 = self.create_publisher(...)
self.pub1.publish(msg)

# With PubSubManager
self.pubsub.create_publisher(MsgType, '/topic1', 10)
self.pubsub.create_publisher(MsgType, '/topic2', 10)
self.pubsub.publish('/topic1', msg)
```

## Design Pattern: JoyBase Inheritance

**Problem:** Repeated joystick setup and mapping code.

**Solution:** Inherit from JoyBase, override `joy_callback`.

```python
class MyController(JoyBase):
    def joy_callback(self, msg):
        # Access predefined mappings
        speed = msg.axes[self.axis_dict['LY']]
        if msg.buttons[self.button_dict['A']]:
            self.do_action()
```
