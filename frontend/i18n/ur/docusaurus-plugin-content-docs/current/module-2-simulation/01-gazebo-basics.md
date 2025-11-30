---
sidebar_position: 2
title: "Gazebo بنیادیات"
description: "روبوٹ سمیولیشن کے لیے Gazebo سیٹ اپ اور استعمال"
---

# Gazebo بنیادیات (Gazebo Basics)

اس سیکشن میں، آپ سمیولیشن ورلڈز بنانا، سینسرز شامل کرنا، اور ROS2 کے ساتھ انٹیگریٹ کرنا سیکھیں گے۔

## سیکھنے کے مقاصد (Learning Objectives)

- اپنی Gazebo ورلڈز بنائیں
- فزکس سمیولیشن کنفیگر کریں
- اپنی سمیولیشن میں سینسرز شامل کریں
- ROS2 سے سمیولیٹڈ روبوٹس کنٹرول کریں

## فزکس کنفیگریشن (Physics Configuration)

Gazebo متعدد فزکس انجنز کو سپورٹ کرتا ہے:

```xml title="physics_config.sdf"
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.81</gravity>
</physics>
```

### فزکس پیرامیٹرز (Physics Parameters)

| پیرامیٹر | تفصیل | عام قدر |
|-----------|-------------|---------------|
| `max_step_size` | سمیولیشن ٹائم سٹیپ | 0.001 (1ms) |
| `real_time_factor` | رفتار ضرب کار | 1.0 (ریئل ٹائم) |
| `gravity` | کشش ثقل ویکٹر | 0 0 -9.81 |

## سینسرز شامل کرنا (Adding Sensors)

### کیمرہ سینسر (Camera Sensor)

```xml title="camera_sensor.sdf"
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>image_raw:=camera/image</remapping>
    </ros>
    <camera_name>front_camera</camera_name>
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```

### LiDAR سینسر

```xml title="lidar_sensor.sdf"
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>
```

## ROS2 انٹیگریشن

### ماڈلز سپان کرنا (Spawning Models)

```bash
# URDF سے ماڈل سپان کریں
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description \
  -entity my_robot \
  -x 0 -y 0 -z 0.5

# فائل سے سپان کریں
ros2 run gazebo_ros spawn_entity.py \
  -file /path/to/model.urdf \
  -entity my_robot
```

### Gazebo-ROS برج ٹاپکس

| Gazebo پلگ ان | ROS2 ٹاپک ٹائپ |
|---------------|-----------------|
| `libgazebo_ros_camera.so` | `sensor_msgs/Image` |
| `libgazebo_ros_ray_sensor.so` | `sensor_msgs/LaserScan` |
| `libgazebo_ros_imu_sensor.so` | `sensor_msgs/Imu` |
| `libgazebo_ros_diff_drive.so` | `geometry_msgs/Twist` |

### سمیولیٹڈ روبوٹ کنٹرول کرنا

```python title="sim_controller.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimController(Node):
    def __init__(self):
        super().__init__('sim_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        cmd = Twist()
        cmd.linear.x = 0.5  # آگے
        cmd.angular.z = 0.1  # موڑ
        self.publisher.publish(cmd)

def main():
    rclpy.init()
    node = SimController()
    rclpy.spin(node)
    rclpy.shutdown()
```

## سمیولیشن کے لیے لانچ فائل

```python title="launch/simulation.launch.py"
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Gazebo لانچ فائل حاصل کریں
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': 'path/to/world.sdf'}.items()
    )

    # روبوٹ سپان کریں
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot'
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_robot,
    ])
```

## خلاصہ (Summary)

- **فزکس کنفیگریشن** سمیولیشن کی درستگی اور رفتار کنٹرول کرتی ہے
- **سینسرز** پلگ انز کے ذریعے شامل ہوتے ہیں جو ROS2 سے برج کرتے ہیں
- **لانچ فائلز** پیچیدہ سمیولیشن سیٹ اپس کو آرکیسٹریٹ کرتی ہیں
- **gazebo_ros** پلگ انز ہموار ROS2 انٹیگریشن فعال کرتے ہیں

**اگلا**: [URDF روبوٹس](./urdf-robots) - اپنے روبوٹ ماڈلز بنائیں۔
