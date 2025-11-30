---
sidebar_position: 4
title: "Python انٹیگریشن (rclpy)"
description: "rclpy کلائنٹ لائبریری کا استعمال کرتے ہوئے Python میں ROS2 ایپلیکیشنز بنانا"
---

# rclpy کے ساتھ Python انٹیگریشن

**rclpy** ROS2 کے لیے سرکاری Python کلائنٹ لائبریری ہے۔ یہ نوڈز، publishers، subscribers، سروسز، ایکشنز، اور مزید بنانے کے لیے Pythonic API فراہم کرتی ہے۔

## سیکھنے کے مقاصد

- ROS2 ڈویلپمنٹ کے لیے rclpy API میں مہارت حاصل کرنا
- اچھی ساخت والے ROS2 Python پیکجز بنانا
- پیرامیٹرز اور launch فائلوں کو سنبھالنا
- دوبارہ قابل استعمال روبوٹ کمپوننٹس بنانا

## rclpy آرکیٹیکچر

rclpy لائبریری ROS2 core (rcl) کو Python میں لپیٹتی ہے:

```text
┌─────────────────────────────────────────┐
│          آپ کا Python کوڈ               │
├─────────────────────────────────────────┤
│              rclpy                       │
│  (Node, Publisher, Subscriber, وغیرہ)    │
├─────────────────────────────────────────┤
│          rcl (C لائبریری)                │
├─────────────────────────────────────────┤
│          rmw (ROS Middleware)           │
├─────────────────────────────────────────┤
│              DDS                        │
└─────────────────────────────────────────┘
```

## ROS2 Python پیکج بنانا

### پیکج کی ساخت

ایک عام ROS2 Python پیکج کی ساخت:

```text
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py
│   ├── my_node.py
│   └── utils.py
├── resource/
│   └── my_robot_pkg
├── test/
│   └── test_my_node.py
├── launch/
│   └── robot.launch.py
├── config/
│   └── params.yaml
├── package.xml
├── setup.py
└── setup.cfg
```

### پیکج بنانا

نیا پیکج بنانے کے لیے:

```bash
# ورک اسپیس میں جائیں
cd ~/ros2_ws/src

# نیا Python پیکج بنائیں
ros2 pkg create --build-type ament_python my_robot_pkg

# پیکج بنائیں
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg

# سیٹ اپ سورس کریں
source install/setup.bash
```

## پیرامیٹرز

ROS2 پیرامیٹرز نوڈز کو runtime پر کنفیگر کرنے کی اجازت دیتے ہیں:

```python title="parameter_node.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    """پیرامیٹرز کا استعمال کرنے والا نوڈ۔"""

    def __init__(self):
        super().__init__('parameter_node')

        # پیرامیٹرز declare کریں
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_speed', 1.0)

        # پیرامیٹرز حاصل کریں
        robot_name = self.get_parameter('robot_name').value
        max_speed = self.get_parameter('max_speed').value

        self.get_logger().info(f'روبوٹ: {robot_name}, زیادہ سے زیادہ رفتار: {max_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch فائلیں

Launch فائلیں متعدد نوڈز کو ایک ساتھ شروع کرنے کی اجازت دیتی ہیں:

```python title="robot.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='sensor_node',
            name='camera',
            parameters=[{'frame_rate': 30}]
        ),
        Node(
            package='my_robot_pkg',
            executable='controller_node',
            name='controller',
            parameters=[{'max_speed': 1.5}]
        ),
    ])
```

## Executors اور Callbacks

rclpy مختلف executor پیٹرنز فراہم کرتی ہے:

```python title="multi_callback_node.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

class MultiCallbackNode(Node):
    """متعدد callbacks والا نوڈ۔"""

    def __init__(self):
        super().__init__('multi_callback_node')

        # متعدد timers بنائیں
        self.timer1 = self.create_timer(1.0, self.timer1_callback)
        self.timer2 = self.create_timer(0.5, self.timer2_callback)

    def timer1_callback(self):
        self.get_logger().info('ٹائمر 1 فائر ہوا')

    def timer2_callback(self):
        self.get_logger().info('ٹائمر 2 فائر ہوا')

def main(args=None):
    rclpy.init(args=args)
    node = MultiCallbackNode()

    # MultiThreadedExecutor برائے ہمزمان callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## خلاصہ

- **rclpy** ROS2 کے لیے سرکاری Python لائبریری ہے
- **پیرامیٹرز** runtime کنفیگریشن کی اجازت دیتے ہیں
- **Launch فائلیں** متعدد نوڈز کو منظم کرتی ہیں
- **Executors** callback execution کو کنٹرول کرتے ہیں

آپ نے ماڈیول 1 مکمل کیا! اگلے ماڈیول میں، ہم روبوٹ سمولیشن کے بارے میں سیکھیں گے۔

**اگلا**: [روبوٹ سمولیشن](../module-2-simulation/) - Gazebo کے ساتھ روبوٹ سمولیشن سیکھیں۔
