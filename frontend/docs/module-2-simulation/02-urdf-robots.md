---
sidebar_position: 3
title: "URDF Robot Descriptions"
description: "Creating robot models with the Unified Robot Description Format"
---

# URDF Robot Descriptions

**URDF** (Unified Robot Description Format) is the standard XML format for describing robots in ROS. It defines the robot's physical structure, kinematics, and visual appearance.

## Learning Objectives

- Understand URDF structure and components
- Create links and joints for robot models
- Add visual and collision geometry
- Generate URDF from xacro templates

## URDF Structure

A URDF file describes a robot as a tree of **links** connected by **joints**:

```text
┌───────────────────────────────────────────────────┐
│                    URDF Robot                      │
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

## Basic URDF Example

```xml title="simple_robot.urdf"
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Base Link -->
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

  <!-- Left Wheel -->
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

  <!-- Left Wheel Joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.175 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

## URDF Components

### Links

Links represent rigid bodies:

```xml
<link name="arm_link">
  <!-- Visual: What you see -->
  <visual>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.04" length="0.5"/>
    </geometry>
    <material name="silver">
      <color rgba="0.7 0.7 0.7 1"/>
    </material>
  </visual>

  <!-- Collision: What physics engine uses -->
  <collision>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.04" length="0.5"/>
    </geometry>
  </collision>

  <!-- Inertial: Mass properties -->
  <inertial>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.02" ixy="0" ixz="0"
             iyy="0.02" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

### Joint Types

| Type | Description | DOF |
|------|-------------|-----|
| `fixed` | No movement | 0 |
| `revolute` | Rotation with limits | 1 |
| `continuous` | Unlimited rotation | 1 |
| `prismatic` | Linear sliding | 1 |
| `floating` | Full 6DOF | 6 |
| `planar` | 2D movement | 2 |

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

## Using Xacro for Reusability

**Xacro** (XML Macro) makes URDF more maintainable:

```xml title="robot.urdf.xacro"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="base_length" value="0.5"/>

  <!-- Wheel Macro -->
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

  <!-- Instantiate wheels -->
  <xacro:wheel prefix="front_left" x_reflect="1" y_reflect="1"/>
  <xacro:wheel prefix="front_right" x_reflect="1" y_reflect="-1"/>
  <xacro:wheel prefix="rear_left" x_reflect="-1" y_reflect="1"/>
  <xacro:wheel prefix="rear_right" x_reflect="-1" y_reflect="-1"/>

</robot>
```

### Processing Xacro

```bash
# Generate URDF from xacro
xacro robot.urdf.xacro > robot.urdf

# Or use ROS2 parameter
ros2 param set /robot_state_publisher robot_description \
  "$(xacro robot.urdf.xacro)"
```

## Visualizing URDF

### Using rviz2

```bash
# Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat robot.urdf)"

# Launch rviz2
rviz2
# Add RobotModel display, set Fixed Frame to "base_link"
```

### Launch File for Visualization

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

## Summary

- **URDF** defines robot structure with links and joints
- **Links** have visual, collision, and inertial properties
- **Joints** connect links with various motion types
- **Xacro** enables reusable, parameterized robot descriptions
- **rviz2** visualizes URDF models

**Next Module**: [NVIDIA Isaac Platform](/docs/module-3-nvidia-isaac) - GPU-accelerated robotics.
