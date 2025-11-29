---
sidebar_position: 3
title: "Isaac ROS Integration"
description: "GPU-accelerated perception and robotics with Isaac ROS"
---

# Isaac ROS Integration

**Isaac ROS** provides GPU-accelerated ROS2 packages that dramatically improve perception and navigation performance. These packages leverage NVIDIA GPUs to run computer vision and AI algorithms in real-time.

## Learning Objectives

- Set up Isaac ROS development environment
- Use GPU-accelerated perception packages
- Integrate Visual SLAM for localization
- Deploy object detection models

## Isaac ROS Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                      Isaac ROS                               │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │
│  │  Visual SLAM    │  │  Object         │  │  DNN        │ │
│  │  (cuVSLAM)      │  │  Detection      │  │  Inference  │ │
│  └────────┬────────┘  └────────┬────────┘  └──────┬──────┘ │
│           │                    │                   │        │
│  ┌─────────────────────────────────────────────────────────┐│
│  │              NVIDIA VPI (Vision Pipeline)               ││
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

## Setting Up Isaac ROS

### Prerequisites

```bash
# System requirements
# - Ubuntu 22.04
# - ROS2 Humble
# - NVIDIA GPU with CUDA support
# - Docker (recommended)

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Using Isaac ROS Docker

```bash
# Clone Isaac ROS common
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Run the Isaac ROS development container
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh

# Inside container, build packages
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

### Native Installation

```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone required packages
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git

# Install dependencies
cd ~/isaac_ros_ws
rosdep install -i -r --from-paths src --rosdistro humble -y

# Build
colcon build --symlink-install
source install/setup.bash
```

## Isaac ROS Visual SLAM

**cuVSLAM** provides GPU-accelerated visual simultaneous localization and mapping:

### Launch Visual SLAM

```bash
# Launch with RealSense camera
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py

# Launch with custom camera
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info
```

### Visual SLAM Configuration

```python title="launch/vslam.launch.py"
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_localization_n_mapping': True,
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'img_jitter_threshold_ms': 34.0,
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'input_left_camera_frame': 'camera_left',
            'input_right_camera_frame': 'camera_right',
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/left/image_raw'),
            ('visual_slam/camera_info_0', '/camera/left/camera_info'),
            ('visual_slam/image_1', '/camera/right/image_raw'),
            ('visual_slam/camera_info_1', '/camera/right/camera_info'),
            ('visual_slam/imu', '/imu/data'),
        ]
    )

    container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    return LaunchDescription([container])
```

### Visual SLAM Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | Robot pose estimate |
| `/visual_slam/tracking/slam_path` | `nav_msgs/Path` | Full trajectory |
| `/visual_slam/tracking/vo_pose` | `geometry_msgs/PoseStamped` | Visual odometry pose |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | System status |

## Isaac ROS Object Detection

GPU-accelerated object detection using TensorRT:

### Supported Models

| Model | Framework | Use Case |
|-------|-----------|----------|
| DetectNet | TAO | General detection |
| YOLOv5 | PyTorch/TensorRT | Fast detection |
| SSD MobileNet | TensorFlow/TensorRT | Mobile deployment |
| Faster R-CNN | PyTorch/TensorRT | High accuracy |

### Running Object Detection

```bash
# Download pre-trained model
mkdir -p /tmp/models
wget -O /tmp/models/ssd_mobilenet.etlt \
  https://api.ngc.nvidia.com/v2/models/nvidia/tao/ssd_mobilenet_v2/versions/1/files/ssd_mobilenet_v2.etlt

# Launch detection node
ros2 launch isaac_ros_object_detection isaac_ros_detectnet.launch.py \
  model_file_path:=/tmp/models/ssd_mobilenet.etlt
```

### Detection Node Configuration

```python title="launch/object_detection.launch.py"
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Encoder node (image preprocessing)
    encoder_node = ComposableNode(
        name='dnn_image_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        parameters=[{
            'input_image_width': 640,
            'input_image_height': 480,
            'network_image_width': 300,
            'network_image_height': 300,
            'image_mean': [0.5, 0.5, 0.5],
            'image_stddev': [0.5, 0.5, 0.5],
        }],
        remappings=[
            ('image', '/camera/image_raw'),
            ('encoded_tensor', 'tensor_pub')
        ]
    )

    # TensorRT inference node
    triton_node = ComposableNode(
        name='triton_node',
        package='isaac_ros_triton',
        plugin='nvidia::isaac_ros::dnn_inference::TritonNode',
        parameters=[{
            'model_name': 'ssd_mobilenet',
            'model_repository_paths': ['/tmp/models'],
            'max_batch_size': 8,
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['input'],
            'output_tensor_names': ['output_tensor'],
            'output_binding_names': ['output'],
        }]
    )

    # DetectNet decoder
    detectnet_decoder_node = ComposableNode(
        name='detectnet_decoder_node',
        package='isaac_ros_detectnet',
        plugin='nvidia::isaac_ros::detectnet::DetectNetDecoderNode',
        parameters=[{
            'label_list': ['person', 'car', 'bicycle'],
            'confidence_threshold': 0.5,
        }]
    )

    container = ComposableNodeContainer(
        name='detectnet_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            encoder_node,
            triton_node,
            detectnet_decoder_node
        ],
        output='screen'
    )

    return LaunchDescription([container])
```

### Detection Output

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
            # Get bounding box
            bbox = detection.bbox
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y
            width = bbox.size_x
            height = bbox.size_y

            # Get class and confidence
            if detection.results:
                result = detection.results[0]
                class_id = result.hypothesis.class_id
                confidence = result.hypothesis.score

                self.get_logger().info(
                    f'Detected {class_id} at ({center_x:.1f}, {center_y:.1f}) '
                    f'with confidence {confidence:.2f}'
                )

def main():
    rclpy.init()
    node = DetectionSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
```

## Isaac ROS AprilTag Detection

GPU-accelerated fiducial marker detection:

```bash
# Launch AprilTag detection
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info
```

### AprilTag Configuration

```python title="launch/apriltag.launch.py"
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    apriltag_node = ComposableNode(
        name='apriltag',
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        parameters=[{
            'size': 0.1,  # Tag size in meters
            'max_tags': 20,
            'tile_size': 4,
        }],
        remappings=[
            ('image', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )

    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[apriltag_node],
        output='screen'
    )

    return LaunchDescription([container])
```

## Isaac ROS with Jetson

Deploying Isaac ROS on Jetson platforms:

### Jetson Setup

```bash
# Install JetPack SDK (includes CUDA, TensorRT, cuDNN)
# Download from NVIDIA website and flash to Jetson

# Install ROS2 Humble on Jetson
sudo apt update
sudo apt install ros-humble-desktop

# Clone Isaac ROS packages
cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Build for Jetson
cd ~/ros2_ws
colcon build --packages-select isaac_ros_visual_slam
```

### Performance Optimization

```python title="launch/optimized_perception.launch.py"
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Optimized perception pipeline for Jetson."""

    # Use NITROS for zero-copy message passing
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            # Enable GPU-direct memory
            'enable_gpu_direct': True,
            # Reduce resolution for Jetson
            'image_width': 640,
            'image_height': 480,
            # Optimize for Jetson Orin
            'num_cameras': 2,
        }]
    )

    object_detection_node = ComposableNode(
        name='object_detection',
        package='isaac_ros_detectnet',
        plugin='nvidia::isaac_ros::detectnet::DetectNetNode',
        parameters=[{
            # Use INT8 precision for speed
            'precision': 'int8',
            # Batch processing
            'max_batch_size': 4,
        }]
    )

    # Run all nodes in single container for efficiency
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded
        composable_node_descriptions=[
            visual_slam_node,
            object_detection_node,
        ],
        output='screen'
    )

    return LaunchDescription([container])
```

## Complete Example: Perception Pipeline

```python title="perception_pipeline.py"
#!/usr/bin/env python3
"""Complete Isaac ROS perception pipeline."""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped

class PerceptionPipeline(Node):
    """Unified perception pipeline using Isaac ROS."""

    def __init__(self):
        super().__init__('perception_pipeline')

        # State variables
        self.current_pose = None
        self.detections = []

        # Subscribe to Visual SLAM odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )

        # Subscribe to object detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detection_callback,
            10
        )

        # Publisher for fused pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot/pose',
            10
        )

        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_perception)

        self.get_logger().info('Perception pipeline initialized')

    def odom_callback(self, msg: Odometry):
        """Handle Visual SLAM odometry."""
        self.current_pose = msg.pose.pose

    def detection_callback(self, msg: Detection2DArray):
        """Handle object detections."""
        self.detections = []
        for det in msg.detections:
            if det.results:
                self.detections.append({
                    'class': det.results[0].hypothesis.class_id,
                    'confidence': det.results[0].hypothesis.score,
                    'bbox': det.bbox
                })

    def process_perception(self):
        """Process and fuse perception data."""
        if self.current_pose is None:
            return

        # Publish current pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = self.current_pose
        self.pose_pub.publish(pose_msg)

        # Log detections
        if self.detections:
            self.get_logger().info(
                f'Pose: ({self.current_pose.position.x:.2f}, '
                f'{self.current_pose.position.y:.2f}), '
                f'Detections: {len(self.detections)}'
            )

def main():
    rclpy.init()
    node = PerceptionPipeline()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

- **Isaac ROS** provides GPU-accelerated ROS2 packages
- **cuVSLAM** enables real-time visual localization
- **DetectNet** offers TensorRT-optimized object detection
- **NITROS** enables zero-copy GPU message passing
- **Jetson** deployment supports edge AI applications

**Next Module**: [Vision-Language-Action Models](/docs/module-4-vla) - AI models that understand language and control robots.
