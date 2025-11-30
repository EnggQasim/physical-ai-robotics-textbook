---
sidebar_position: 1
title: "ROS2 کا جائزہ"
description: "ROS2 کا تعارف - روبوٹ آپریٹنگ سسٹم 2"
---

# ROS2 بنیادی باتیں

فزیکل اے آئی کتاب کے ماڈیول 1 میں خوش آمدید۔ اس ماڈیول میں، آپ **روبوٹ آپریٹنگ سسٹم 2 (ROS2)** میں مہارت حاصل کریں گے، جو جدید روبوٹکس سافٹ ویئر ڈویلپمنٹ کی بنیاد ہے۔

## سیکھنے کے مقاصد

اس ماڈیول کے اختتام تک، آپ قابل ہوں گے:

- ROS2 آرکیٹیکچر اور ڈیزائن اصولوں کو سمجھنا
- Python میں ROS2 نوڈز بنانا اور منظم کرنا
- ٹاپکس کے ساتھ publisher-subscriber کمیونیکیشن کا نفاذ
- سروسز کے ساتھ request-response پیٹرنز بنانا
- ایکشنز کے ساتھ طویل عرصے کے ٹاسکس سنبھالنا
- اپنے پروجیکٹس کے لیے ROS2 پیکجز کی ساخت بنانا

## ROS2 کیا ہے؟

**ROS2** (Robot Operating System 2) روایتی معنوں میں آپریٹنگ سسٹم نہیں ہے۔ یہ ایک **مڈل ویئر فریم ورک** ہے جو فراہم کرتا ہے:

1. **کمیونیکیشن پرت** (Communication Layer): عمل کے درمیان رابطے کے اوزار
2. **ہارڈویئر ایبسٹریکشن** (Hardware Abstraction): سینسرز اور ایکچویٹرز کے لیے معیاری انٹرفیس
3. **ڈیوائس ڈرائیورز** (Device Drivers): عام روبوٹکس ہارڈویئر کے لیے پہلے سے بنے ہوئے ڈرائیورز
4. **لائبریریز** (Libraries): نیویگیشن، ادراک، ہیرا پھیری کے الگورتھم
5. **ٹولز** (Tools): تصویری نمائش، ڈیبگنگ، سمولیشن انٹیگریشن

### ROS بمقابلہ ROS2

ROS2 اصل ROS کی مکمل ری ڈیزائن ہے، جو اہم محدودیات کو دور کرتی ہے:

| پہلو | ROS1 | ROS2 |
|------|------|------|
| کمیونیکیشن | Custom (TCPROS) | DDS معیار |
| ریئل ٹائم | سپورٹ نہیں | سپورٹ موجود |
| سیکیورٹی | بلٹ ان نہیں | DDS-Security |
| ملٹی روبوٹ | مشکل | مقامی سپورٹ |
| پلیٹ فارمز | صرف Linux | Linux، Windows، macOS |
| Python | Python 2 | Python 3 |

### ROS2 اہم کیوں ہے؟

ROS2 انڈسٹری کا معیار بن گیا ہے کیونکہ:

- **اوپن سورس** (Open Source): مفت استعمال، ترمیم، اور تقسیم
- **بڑا ایکو سسٹم** (Large Ecosystem): ہزاروں پیکجز اور فعال کمیونٹی
- **صنعتی اپنانا** (Industry Adoption): بڑی روبوٹکس کمپنیوں کا استعمال
- **جدید ڈیزائن** (Modern Design): پروڈکشن کے لیے بنایا گیا، صرف تحقیق کے لیے نہیں

## بنیادی تصورات کا جائزہ

ROS2 کئی اہم تصورات کے گرد بنایا گیا ہے جن کی ہم تفصیل سے کھوج کریں گے:

### 1. نوڈز (Nodes)

**نوڈز** ROS2 ایپلیکیشنز کے بنیادی تعمیراتی بلاکس ہیں۔ ہر نوڈ ایک پروسیس ہے جو ایک مخصوص کام انجام دیتا ہے۔

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        self.get_logger().info('نوڈ شروع ہو گیا!')
```

نوڈز کو مائیکرو سروسز کی طرح سمجھیں - ہر ایک ایک ذمہ داری سنبھالتا ہے۔

### 2. ٹاپکس (Topics)

**ٹاپکس** publish-subscribe کمیونیکیشن کو فعال کرتے ہیں۔ Publishers پیغامات ٹاپک پر بھیجتے ہیں، اور Subscribers انہیں وصول کرتے ہیں۔

```text
┌──────────────┐      /robot/velocity      ┌──────────────┐
│   Joystick   │ ─────────────────────────► │    Robot     │
│   Node       │      (Publisher)           │   Controller │
└──────────────┘                            │   (Subscriber)│
                                            └──────────────┘
```

### 3. سروسز (Services)

**سروسز** request-response کمیونیکیشن فراہم کرتی ہیں۔ ایک کلائنٹ درخواست بھیجتا ہے، اور سرور جواب دیتا ہے۔

```text
┌──────────────┐                            ┌──────────────┐
│    Client    │ ──── Request ────────────► │    Server    │
│              │ ◄─── Response ───────────  │              │
└──────────────┘                            └──────────────┘
```

### 4. ایکشنز (Actions)

**ایکشنز** طویل عرصے کے ٹاسکس کو فیڈبیک کے ساتھ سنبھالتے ہیں۔ یہ سروسز کو پیش رفت کی تازہ کاریوں کے لیے ٹاپکس کے ساتھ جوڑتے ہیں۔

```text
┌──────────────┐      Goal       ┌──────────────┐
│    Client    │ ───────────────►│    Server    │
│              │ ◄── Feedback ─── │              │
│              │ ◄── Result ───── │              │
└──────────────┘                  └──────────────┘
```

## ROS2 آرکیٹیکچر

ROS2 آرکیٹیکچر کئی پرتوں پر مشتمل ہے:

```text
┌─────────────────────────────────────────────────────────┐
│                   آپ کی ایپلیکیشن                         │
│              (Nodes, Packages, Launch Files)            │
├─────────────────────────────────────────────────────────┤
│                   ROS2 Client Libraries                  │
│              (rclpy, rclcpp, rclnodejs)                 │
├─────────────────────────────────────────────────────────┤
│                      ROS2 Core (rcl)                    │
│              (Node, Publisher, Subscriber APIs)         │
├─────────────────────────────────────────────────────────┤
│                    ROS Middleware (rmw)                  │
│              (DDS پر ایبسٹریکشن)                          │
├─────────────────────────────────────────────────────────┤
│                  DDS Implementation                      │
│        (Fast-DDS, Cyclone DDS, RTI Connext)             │
└─────────────────────────────────────────────────────────┘
```

### ڈیٹا ڈسٹری بیوشن سروس (DDS)

ROS2 اپنی کمیونیکیشن پرت کے طور پر **DDS** (Data Distribution Service) استعمال کرتا ہے:

- **غیر مرکزی** (Decentralized): کوئی مرکزی ماسٹر نہیں (ROS1 کے roscore کے برعکس)
- **خودکار دریافت** (Automatic Discovery): نوڈز خود بخود ایک دوسرے کو ڈھونڈ لیتے ہیں
- **Quality of Service (QoS)**: قابل ترتیب reliability، durability، deadline
- **ریئل ٹائم قابل** (Real-time Capable): یقینی پیغام رسانی

## اپنا ماحول ترتیب دینا

### پیش شرطیں

شروع کرنے سے پہلے، یقینی بنائیں کہ آپ کے پاس:

1. **Ubuntu 22.04** (تجویز کردہ) یا Windows/macOS
2. **Python 3.10+**
3. **ROS2 Humble** یا نیا

### انسٹالیشن

Ubuntu 22.04 پر، ROS2 Humble انسٹال کریں:

```bash
# ROS2 ریپوزٹری شامل کریں
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble انسٹال کریں
sudo apt update
sudo apt install ros-humble-desktop

# سیٹ اپ اسکرپٹ سورس کریں
source /opt/ros/humble/setup.bash
```

### انسٹالیشن کی تصدیق

اپنی انسٹالیشن کی جانچ کریں:

```bash
# ROS2 ورژن چیک کریں
ros2 --version

# ڈیمو چلائیں
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
```

آپ کو پیغامات publish اور receive ہوتے نظر آنے چاہئیں۔

## ماڈیول کی ساخت

یہ ماڈیول درج ذیل حصوں میں منظم ہے:

1. **[نوڈز اور ٹاپکس](./nodes-topics)** - Publisher-subscriber کمیونیکیشن
2. **[سروسز اور ایکشنز](./services-actions)** - Request-response اور طویل عرصے کے ٹاسکس
3. **[Python انٹیگریشن](./python-rclpy)** - rclpy کے ساتھ ROS2 ایپلیکیشنز بنانا

ہر سیکشن میں شامل ہے:
- تصوراتی وضاحتیں
- کام کرنے والی کوڈ مثالیں
- عملی مشقیں

## خلاصہ

اس جائزے میں، آپ نے سیکھا:

- ROS2 روبوٹکس ڈویلپمنٹ کے لیے ایک مڈل ویئر فریم ورک ہے
- یہ کمیونیکیشن، ایبسٹریکشن، اور ٹولز فراہم کرتا ہے
- بنیادی تصورات میں نوڈز، ٹاپکس، سروسز، اور ایکشنز شامل ہیں
- ROS2 غیر مرکزی، ریئل ٹائم کمیونیکیشن کے لیے DDS استعمال کرتا ہے

**اگلا**: [نوڈز اور ٹاپکس](./nodes-topics) - Publisher-subscriber پیٹرن میں غوطہ لگائیں۔
