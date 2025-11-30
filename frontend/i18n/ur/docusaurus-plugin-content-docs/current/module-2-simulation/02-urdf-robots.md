---
sidebar_position: 3
title: "URDF روبوٹ ڈسکرپشنز"
description: "یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ کے ساتھ روبوٹ ماڈلز بنانا"
---

# URDF روبوٹ ڈسکرپشنز

**URDF** (Unified Robot Description Format) ROS میں روبوٹس کی وضاحت کے لیے معیاری XML فارمیٹ ہے۔ یہ روبوٹ کی جسمانی ساخت، کائنیمیٹکس، اور بصری ظاہری شکل کی وضاحت کرتا ہے۔

## سیکھنے کے مقاصد (Learning Objectives)

- URDF ڈھانچہ اور اجزاء سمجھیں
- روبوٹ ماڈلز کے لیے لنکس اور جوائنٹس بنائیں
- بصری اور ٹکراؤ جیومیٹری شامل کریں
- xacro ٹیمپلیٹس سے URDF بنائیں

## URDF ڈھانچہ (URDF Structure)

URDF فائل روبوٹ کو **لنکس (Links)** کے درخت کے طور پر بیان کرتی ہے جو **جوائنٹس (Joints)** سے جڑے ہیں:

```text
┌───────────────────────────────────────────────────┐
│                    URDF روبوٹ                      │
├───────────────────────────────────────────────────┤
│                                                    │
│     base_link ──(fixed)── sensor_link             │
│         │                                          │
│    (revolute)                                      │
│         │                                          │
│    left_wheel    right_wheel                      │
│    (continuous)  (continuous)                      │
│                                                    │
└───────────────────────────────────────────────────┘
```

## بنیادی URDF مثال

```xml title="simple_robot.urdf"
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- بیس لنک -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0"
               iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- بائیں پہیہ -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- بائیں پہیے کا جوائنٹ -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.175 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

## URDF اجزاء (URDF Components)

### لنکس (Links)

لنکس سخت باڈیز کی نمائندگی کرتے ہیں:

```xml
<link name="arm_link">
  <!-- بصری: جو آپ دیکھتے ہیں -->
  <visual>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.04" length="0.5"/>
    </geometry>
    <material name="silver">
      <color rgba="0.7 0.7 0.7 1"/>
    </material>
  </visual>

  <!-- ٹکراؤ: جو فزکس انجن استعمال کرتا ہے -->
  <collision>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.04" length="0.5"/>
    </geometry>
  </collision>

  <!-- جڑتی: کمیت کی خصوصیات -->
  <inertial>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.02" ixy="0" ixz="0"
             iyy="0.02" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

### جوائنٹ اقسام (Joint Types)

| قسم | تفصیل | DOF |
|------|-------------|-----|
| `fixed` | کوئی حرکت نہیں | 0 |
| `revolute` | حدود کے ساتھ گردش | 1 |
| `continuous` | لامحدود گردش | 1 |
| `prismatic` | لکیری سلائیڈنگ | 1 |
| `floating` | مکمل 6DOF | 6 |
| `planar` | 2D حرکت | 2 |

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57"
         effort="100" velocity="1.0"/>
</joint>
```

## دوبارہ استعمال کے لیے Xacro

**Xacro** (XML Macro) URDF کو زیادہ قابل دیکھ بھال بناتا ہے:

```xml title="robot.urdf.xacro"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- خصوصیات -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="base_length" value="0.5"/>

  <!-- پہیے کا میکرو -->
  <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x_reflect * base_length/4} ${y_reflect * 0.15} 0"
              rpy="1.5708 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- پہیوں کی مثالیں بنائیں -->
  <xacro:wheel prefix="front_left" x_reflect="1" y_reflect="1"/>
  <xacro:wheel prefix="front_right" x_reflect="1" y_reflect="-1"/>
  <xacro:wheel prefix="rear_left" x_reflect="-1" y_reflect="1"/>
  <xacro:wheel prefix="rear_right" x_reflect="-1" y_reflect="-1"/>

</robot>
```

### Xacro پروسیسنگ

```bash
# xacro سے URDF بنائیں
xacro robot.urdf.xacro > robot.urdf

# یا ROS2 پیرامیٹر استعمال کریں
ros2 param set /robot_state_publisher robot_description \
  "$(xacro robot.urdf.xacro)"
```

## URDF ویژولائز کرنا

### rviz2 استعمال کرنا

```bash
# روبوٹ سٹیٹ پبلشر شروع کریں
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat robot.urdf)"

# rviz2 شروع کریں
rviz2
# RobotModel ڈسپلے شامل کریں، فکسڈ فریم کو "base_link" سیٹ کریں
```

### ویژولائزیشن کے لیے لانچ فائل

```python title="launch/display.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf', 'robot.urdf.xacro'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file])
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        ),
    ])
```

## خلاصہ (Summary)

- **URDF** روبوٹ ڈھانچے کو لنکس اور جوائنٹس کے ساتھ بیان کرتا ہے
- **لنکس** میں بصری، ٹکراؤ، اور جڑتی خصوصیات ہوتی ہیں
- **جوائنٹس** لنکس کو مختلف حرکت کی اقسام سے جوڑتے ہیں
- **Xacro** دوبارہ استعمال کے قابل، پیرامیٹرائزڈ روبوٹ ڈسکرپشنز فعال کرتا ہے
- **rviz2** URDF ماڈلز ویژولائز کرتا ہے

**اگلا ماڈیول**: [NVIDIA Isaac پلیٹ فارم](/docs/module-3-nvidia-isaac) - GPU ایکسلریٹڈ روبوٹکس۔
