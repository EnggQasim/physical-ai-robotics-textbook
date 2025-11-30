---
sidebar_position: 2
title: "نوڈز اور ٹاپکس"
description: "ROS2 نوڈز اور ٹاپکس کے ساتھ publish-subscribe پیٹرن کو سمجھنا"
---

# ROS2 نوڈز اور ٹاپکس

اس سیکشن میں، آپ ROS2 کے بنیادی تعمیراتی بلاکس کے بارے میں سیکھیں گے: **نوڈز** (nodes) اور **ٹاپکس** (topics)۔ یہ تصورات کسی بھی ROS2 ایپلیکیشن کی ریڑھ کی ہڈی بناتے ہیں۔

## سیکھنے کے مقاصد

- Python میں ROS2 نوڈز بنانا
- Publishers اور Subscribers کا نفاذ
- پیغام کی اقسام اور QoS سیٹنگز کو سمجھنا
- ایک مکمل pub-sub سسٹم بنانا

## نوڈز کیا ہیں؟

ایک **نوڈ** (node) ROS2 میں ایک واحد مقصد کا پروسیس ہے۔ ہر نوڈ کو ایک کام اچھی طرح کرنا چاہیے، Unix فلسفے کی پیروی کرتے ہوئے۔

### نوڈ کی خصوصیات

- **واحد ذمہ داری** (Single Responsibility): ہر نوڈ ایک کام
- **ماڈیولر** (Modular): تبدیل یا اپ گریڈ کرنا آسان
- **دوبارہ قابل استعمال** (Reusable): مختلف روبوٹ کنفیگریشنز میں استعمال
- **ڈیبگ کے قابل** (Debuggable): الگ تھلگ ناکامیاں، آسان ٹیسٹنگ

### نوڈ آرکیٹیکچر کی مثال

ایک عام روبوٹ میں یہ نوڈز ہو سکتے ہیں:

```text
┌─────────────────────────────────────────────────────────┐
│                     روبوٹ سسٹم                           │
├─────────────────────────────────────────────────────────┤
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐   │
│  │ Camera  │  │ LiDAR   │  │ Motor   │  │ Battery │   │
│  │  نوڈ    │  │  نوڈ    │  │ Control │  │ Monitor │   │
│  └─────────┘  └─────────┘  └─────────┘  └─────────┘   │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐   │
│  │ Object  │  │  SLAM   │  │  Path   │  │ Safety  │   │
│  │Detector │  │  نوڈ    │  │ Planner │  │  نوڈ    │   │
│  └─────────┘  └─────────┘  └─────────┘  └─────────┘   │
└─────────────────────────────────────────────────────────┘
```

## اپنا پہلا نوڈ بنانا

آئیے Python میں ایک سادہ ROS2 نوڈ بنائیں:

```python title="minimal_node.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    """ایک کم سے کم ROS2 نوڈ جو پیغام لاگ کرتا ہے۔"""

    def __init__(self):
        # نوڈ کو نام کے ساتھ initialize کریں
        super().__init__('minimal_node')

        # لاگ کریں کہ نوڈ شروع ہو گیا ہے
        self.get_logger().info('کم سے کم نوڈ شروع ہو گیا ہے!')

        # ایک ٹائمر بنائیں جو ہر 1 سیکنڈ میں فائر ہو
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """ہر بار ٹائمر فائر ہونے پر کال ہوتا ہے۔"""
        self.counter += 1
        self.get_logger().info(f'ٹائمر {self.counter} بار فائر ہوا')

def main(args=None):
    # ROS2 کو initialize کریں
    rclpy.init(args=args)

    # نوڈ بنائیں اور spin کریں
    node = MinimalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # صفائی کریں
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### نوڈ لائف سائیکل

ہر ROS2 نوڈ اس لائف سائیکل کی پیروی کرتا ہے:

1. **Initialize**: `rclpy.init()` - ROS2 context سیٹ اپ کریں
2. **Create**: اپنی نوڈ کلاس کی مثال بنائیں
3. **Spin**: `rclpy.spin()` - callbacks پروسیس کریں
4. **Shutdown**: `rclpy.shutdown()` - وسائل صاف کریں

## ٹاپکس کیا ہیں؟

**ٹاپکس** (topics) پیغام رسانی کے لیے نامزد بسیں ہیں۔ یہ **publish-subscribe** پیٹرن کا نفاذ کرتے ہیں:

- **Publishers** ٹاپک پر پیغامات بھیجتے ہیں
- **Subscribers** ٹاپک سے پیغامات وصول کرتے ہیں
- **متعدد سے متعدد** (Multiple-to-multiple): کتنے بھی publishers اور subscribers

![ROS2 Publisher-Subscriber Pattern](/img/generated/ros2/pubsub-diagram.svg)
*تصویر: ROS2 Publisher-Subscriber کمیونیکیشن پیٹرن - Publishers ڈیٹا نامزد ٹاپک پر بھیجتے ہیں، اور subscribers اس ٹاپک سے ڈیٹا وصول کرتے ہیں۔*

### ٹاپک کی خصوصیات

- **غیر ہم آہنگ** (Asynchronous): Publishers subscribers کا انتظار نہیں کرتے
- **ڈیکپلڈ** (Decoupled): Publishers اور subscribers ایک دوسرے کو نہیں جانتے
- **ٹائپ شدہ** (Typed): ہر ٹاپک کی ایک مخصوص پیغام کی قسم ہوتی ہے
- **نامزد** (Named): ٹاپکس کے منفرد سٹرنگ نام ہوتے ہیں (مثلاً `/robot/velocity`)

## پیغامات کو سمجھنا

پیغامات ٹاپکس پر بھیجے جانے والے ڈیٹا کی ساخت کی وضاحت کرتے ہیں۔ ROS2 بہت سے معیاری پیغامات فراہم کرتا ہے:

### عام پیغام کی اقسام

| پیکج | پیغام | تفصیل |
|------|-------|-------|
| std_msgs | String | سادہ سٹرنگ |
| std_msgs | Int32, Float64 | عددی اقسام |
| geometry_msgs | Twist | لکیری اور زاویہ دار رفتار |
| geometry_msgs | Pose | پوزیشن اور واقفیت |
| sensor_msgs | Image | کیمرے کی تصاویر |
| sensor_msgs | LaserScan | LiDAR ڈیٹا |

### مثال: Twist پیغام

`Twist` پیغام عام طور پر روبوٹ رفتار کے کمانڈز کے لیے استعمال ہوتا ہے:

```python
from geometry_msgs.msg import Twist

# رفتار کمانڈ بنائیں
vel = Twist()
vel.linear.x = 0.5   # آگے کی رفتار (m/s)
vel.linear.y = 0.0   # پہلو کی رفتار
vel.linear.z = 0.0   # عمودی رفتار
vel.angular.x = 0.0  # Roll rate
vel.angular.y = 0.0  # Pitch rate
vel.angular.z = 0.1  # Yaw rate (rad/s)
```

## Publisher بنانا

آئیے ایک نوڈ بنائیں جو رفتار کے کمانڈز publish کرتا ہے:

```python title="velocity_publisher.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    """رفتار کے کمانڈز publish کرتا ہے تاکہ روبوٹ دائرے میں حرکت کرے۔"""

    def __init__(self):
        super().__init__('velocity_publisher')

        # Publisher بنائیں
        # Arguments: پیغام کی قسم، ٹاپک کا نام، queue سائز
        self.publisher = self.create_publisher(
            Twist,           # پیغام کی قسم
            'cmd_vel',       # ٹاپک کا نام
            10               # Queue سائز
        )

        # 10 Hz پر publish کرنے کے لیے ٹائمر
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.get_logger().info('رفتار publisher شروع ہو گیا')

    def publish_velocity(self):
        """رفتار کمانڈ publish کریں۔"""
        msg = Twist()

        # 0.5 m/s پر آگے بڑھیں جبکہ 0.3 rad/s پر گھومیں
        msg.linear.x = 0.5
        msg.angular.z = 0.3

        # پیغام publish کریں
        self.publisher.publish(msg)

        self.get_logger().debug(
            f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber بنانا

اب آئیے ایک نوڈ بنائیں جو رفتار کے کمانڈز subscribe کرتا ہے:

```python title="velocity_subscriber.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocitySubscriber(Node):
    """رفتار کے کمانڈز subscribe کرتا ہے اور انہیں لاگ کرتا ہے۔"""

    def __init__(self):
        super().__init__('velocity_subscriber')

        # Subscription بنائیں
        self.subscription = self.create_subscription(
            Twist,                    # پیغام کی قسم
            'cmd_vel',                # ٹاپک کا نام
            self.velocity_callback,   # Callback فنکشن
            10                        # Queue سائز
        )

        self.get_logger().info('رفتار subscriber شروع ہو گیا')

    def velocity_callback(self, msg: Twist):
        """رفتار پیغام ملنے پر کال ہوتا ہے۔"""
        self.get_logger().info(
            f'رفتار ملی: '
            f'linear.x={msg.linear.x:.2f}, '
            f'angular.z={msg.angular.z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VelocitySubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS)

QoS سیٹنگز کنٹرول کرتی ہیں کہ پیغامات کیسے ڈیلیور ہوتے ہیں:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Custom QoS کی وضاحت کریں
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # یقینی ڈیلیوری
    history=HistoryPolicy.KEEP_LAST,         # آخری N پیغامات رکھیں
    depth=10                                  # Queue سائز
)

# Publisher میں QoS استعمال کریں
self.publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile)
```

### عام QoS پری سیٹس

| پری سیٹ | استعمال کا معاملہ |
|---------|------------------|
| Sensor Data | Best effort، volatile (تیز، گر سکتا ہے) |
| Services | Reliable، volatile (یقینی لیکن محفوظ نہیں) |
| Parameters | Reliable، transient local (دیر سے آنے والے subscribers کے لیے محفوظ) |

## مثال چلانا

اپنے publisher اور subscriber کی جانچ کرنے کے لیے:

```bash
# ٹرمینل 1: Publisher چلائیں
python3 velocity_publisher.py

# ٹرمینل 2: Subscriber چلائیں
python3 velocity_subscriber.py

# ٹرمینل 3: ٹاپکس کی فہرست دیکھیں اور پیغامات echo کریں
ros2 topic list
ros2 topic echo /cmd_vel
```

## ROS2 CLI ٹولز

ٹاپکس کے ساتھ کام کرنے کے لیے مفید کمانڈز:

```bash
# تمام فعال ٹاپکس کی فہرست
ros2 topic list

# ٹاپک کی معلومات دکھائیں
ros2 topic info /cmd_vel

# پیغامات Echo کریں (CLI سے subscribe)
ros2 topic echo /cmd_vel

# CLI سے publish کریں
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# Publishing rate چیک کریں
ros2 topic hz /cmd_vel

# پیغام کی قسم کی تعریف دکھائیں
ros2 interface show geometry_msgs/msg/Twist
```

## مشقیں

1. **مشق 1**: Publisher میں ترمیم کریں تاکہ آپ کے نام کے ساتھ `String` پیغام publish ہو
2. **مشق 2**: ایک subscriber بنائیں جو گنتا ہے کہ کتنے پیغامات ملے
3. **مشق 3**: ایک سسٹم بنائیں جہاں ایک نوڈ بے ترتیب نمبر publish کرے اور دوسرا running average حساب کرے

## خلاصہ

اس سیکشن میں، آپ نے سیکھا:

- **نوڈز** واحد مقصد کے پروسیسز ہیں جو ROS2 کے ذریعے بات چیت کرتے ہیں
- **ٹاپکس** publish-subscribe کمیونیکیشن کو فعال کرتے ہیں
- **پیغامات** تبادلہ شدہ ڈیٹا کی ساخت کی وضاحت کرتے ہیں
- **QoS** سیٹنگز ڈیلیوری کی ضمانتوں کو کنٹرول کرتی ہیں
- **CLI ٹولز** سسٹم کو ڈیبگ اور معائنہ کرنے میں مدد کرتے ہیں

**اگلا**: [سروسز اور ایکشنز](./services-actions) - ROS2 میں Request-response پیٹرنز۔
