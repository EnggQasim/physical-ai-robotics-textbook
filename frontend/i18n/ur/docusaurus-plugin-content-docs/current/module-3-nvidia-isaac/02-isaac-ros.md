---
sidebar_position: 3
title: "Isaac ROS انٹیگریشن"
description: "Isaac ROS کے ساتھ GPU ایکسلریٹڈ پرسیپشن اور روبوٹکس"
---

# Isaac ROS انٹیگریشن

**Isaac ROS** GPU ایکسلریٹڈ ROS2 پیکجز فراہم کرتا ہے جو پرسیپشن اور نیویگیشن کی کارکردگی کو ڈرامائی طور پر بہتر کرتے ہیں۔ یہ پیکجز NVIDIA GPUs کا فائدہ اٹھاتے ہیں تاکہ کمپیوٹر ویژن اور AI الگورتھم ریئل ٹائم میں چلیں۔

## سیکھنے کے مقاصد (Learning Objectives)

- Isaac ROS ڈیولپمنٹ انوائرنمنٹ سیٹ اپ کریں
- GPU ایکسلریٹڈ پرسیپشن پیکجز استعمال کریں
- لوکلائزیشن کے لیے ویژول SLAM انٹیگریٹ کریں
- آبجیکٹ ڈیٹیکشن ماڈلز ڈیپلائے کریں

## Isaac ROS آرکیٹیکچر

```text
┌─────────────────────────────────────────────────────────────┐
│                      Isaac ROS                               │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │
│  │  ویژول SLAM    │  │  آبجیکٹ         │  │  DNN        │ │
│  │  (cuVSLAM)      │  │  ڈیٹیکشن        │  │  انفرنس    │ │
│  └────────┬────────┘  └────────┬────────┘  └──────┬──────┘ │
│           │                    │                   │        │
│  ┌─────────────────────────────────────────────────────────┐│
│  │              NVIDIA VPI (ویژن پائپ لائن)               ││
│  └─────────────────────────────────────────────────────────┘│
│           │                    │                   │        │
│  ┌─────────────────────────────────────────────────────────┐│
│  │                    CUDA / TensorRT                      ││
│  └─────────────────────────────────────────────────────────┘│
│           │                    │                   │        │
│  ┌─────────────────────────────────────────────────────────┐│
│  │                    NVIDIA GPU                           ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

## Isaac ROS سیٹ اپ کرنا

### پیش شرائط

```bash
# سسٹم تقاضے
# - Ubuntu 22.04
# - ROS2 Humble
# - CUDA سپورٹ والا NVIDIA GPU
# - Docker (تجویز کردہ)

# NVIDIA Container Toolkit انسٹال کریں
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Isaac ROS Docker استعمال کرنا

```bash
# Isaac ROS common کلون کریں
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Isaac ROS ڈیولپمنٹ کنٹینر چلائیں
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh

# کنٹینر کے اندر، پیکجز بلڈ کریں
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

## Isaac ROS ویژول SLAM

**cuVSLAM** GPU ایکسلریٹڈ ویژول سائیملٹینیئس لوکلائزیشن اینڈ میپنگ فراہم کرتا ہے:

### ویژول SLAM شروع کریں

```bash
# RealSense کیمرے کے ساتھ شروع کریں
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py

# کسٹم کیمرے کے ساتھ شروع کریں
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info
```

### ویژول SLAM ٹاپکس

| ٹاپک | ٹائپ | تفصیل |
|-------|------|-------------|
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | روبوٹ پوز اندازہ |
| `/visual_slam/tracking/slam_path` | `nav_msgs/Path` | مکمل ٹریجیکٹری |
| `/visual_slam/tracking/vo_pose` | `geometry_msgs/PoseStamped` | ویژول اوڈومیٹری پوز |

## Isaac ROS آبجیکٹ ڈیٹیکشن

TensorRT کا استعمال کرتے ہوئے GPU ایکسلریٹڈ آبجیکٹ ڈیٹیکشن:

### سپورٹ شدہ ماڈلز

| ماڈل | فریم ورک | استعمال |
|-------|-----------|----------|
| DetectNet | TAO | عمومی ڈیٹیکشن |
| YOLOv5 | PyTorch/TensorRT | تیز ڈیٹیکشن |
| SSD MobileNet | TensorFlow/TensorRT | موبائل ڈیپلائمنٹ |
| Faster R-CNN | PyTorch/TensorRT | اعلی درستگی |

### آبجیکٹ ڈیٹیکشن چلانا

```bash
# پری ٹرینڈ ماڈل ڈاؤن لوڈ کریں
mkdir -p /tmp/models
wget -O /tmp/models/ssd_mobilenet.etlt \
  https://api.ngc.nvidia.com/v2/models/nvidia/tao/ssd_mobilenet_v2/versions/1/files/ssd_mobilenet_v2.etlt

# ڈیٹیکشن نوڈ شروع کریں
ros2 launch isaac_ros_object_detection isaac_ros_detectnet.launch.py \
  model_file_path:=/tmp/models/ssd_mobilenet.etlt
```

### ڈیٹیکشن آؤٹ پٹ

```python title="detection_subscriber.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')

        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        for detection in msg.detections:
            # باؤنڈنگ باکس حاصل کریں
            bbox = detection.bbox
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y
            width = bbox.size_x
            height = bbox.size_y

            # کلاس اور کانفیڈنس حاصل کریں
            if detection.results:
                result = detection.results[0]
                class_id = result.hypothesis.class_id
                confidence = result.hypothesis.score

                self.get_logger().info(
                    f'{class_id} ملا ({center_x:.1f}, {center_y:.1f}) پر '
                    f'کانفیڈنس {confidence:.2f} کے ساتھ'
                )

def main():
    rclpy.init()
    node = DetectionSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
```

## Jetson کے ساتھ Isaac ROS

Jetson پلیٹ فارمز پر Isaac ROS ڈیپلائے کرنا:

### Jetson سیٹ اپ

```bash
# JetPack SDK انسٹال کریں (CUDA، TensorRT، cuDNN شامل)
# NVIDIA ویب سائٹ سے ڈاؤن لوڈ کریں اور Jetson پر فلیش کریں

# Jetson پر ROS2 Humble انسٹال کریں
sudo apt update
sudo apt install ros-humble-desktop

# Isaac ROS پیکجز کلون کریں
cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Jetson کے لیے بلڈ کریں
cd ~/ros2_ws
colcon build --packages-select isaac_ros_visual_slam
```

### کارکردگی آپٹیمائزیشن

```python title="launch/optimized_perception.launch.py"
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Jetson کے لیے آپٹیمائزڈ پرسیپشن پائپ لائن۔"""

    # زیرو کاپی میسج پاسنگ کے لیے NITROS استعمال کریں
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            # GPU-direct میموری فعال کریں
            'enable_gpu_direct': True,
            # Jetson کے لیے ریزولوشن کم کریں
            'image_width': 640,
            'image_height': 480,
            # Jetson Orin کے لیے آپٹیمائز کریں
            'num_cameras': 2,
        }]
    )

    # کارکردگی کے لیے تمام نوڈز سنگل کنٹینر میں چلائیں
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # ملٹی تھریڈڈ
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    return LaunchDescription([container])
```

## خلاصہ (Summary)

- **Isaac ROS** GPU ایکسلریٹڈ ROS2 پیکجز فراہم کرتا ہے
- **cuVSLAM** ریئل ٹائم ویژول لوکلائزیشن فعال کرتا ہے
- **DetectNet** TensorRT آپٹیمائزڈ آبجیکٹ ڈیٹیکشن پیش کرتا ہے
- **NITROS** زیرو کاپی GPU میسج پاسنگ فعال کرتا ہے
- **Jetson** ڈیپلائمنٹ ایج AI ایپلیکیشنز سپورٹ کرتی ہے

**اگلا ماڈیول**: [ویژن-لینگویج-ایکشن ماڈلز](/docs/module-4-vla) - AI ماڈلز جو زبان سمجھتے ہیں اور روبوٹس کنٹرول کرتے ہیں۔
