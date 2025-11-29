---
sidebar_position: 1
title: "Simulation Overview"
description: "Introduction to robot simulation with Gazebo and physics engines"
---

# Robot Simulation

Welcome to Module 2. Simulation is essential for robotics development - it lets you test algorithms safely, generate training data, and iterate quickly without expensive hardware.

## Learning Objectives

- Understand why simulation matters for robotics
- Set up Gazebo simulation environment
- Create robot models with URDF
- Integrate ROS2 with simulated robots

## Why Simulate?

Simulation offers critical advantages:

| Benefit | Description |
|---------|-------------|
| **Safety** | Test dangerous scenarios without risk |
| **Cost** | No hardware damage from bugs |
| **Speed** | Run faster than real-time |
| **Data** | Generate unlimited training data |
| **Parallelization** | Run hundreds of instances |

### The Simulation-to-Reality Gap

Real robots differ from simulated ones:

```text
┌─────────────────────────────────────────────────────────┐
│                    Sim-to-Real Gap                       │
├─────────────────────────────────────────────────────────┤
│  Simulation          │          Reality                  │
│  ─────────────       │          ───────                  │
│  • Perfect sensors   │  • Noise, drift, failures        │
│  • Ideal physics     │  • Friction, flexibility         │
│  • Known state       │  • Partial observability         │
│  • Deterministic     │  • Stochastic effects            │
└─────────────────────────────────────────────────────────┘
```

Domain randomization and system identification help bridge this gap.

## Gazebo Overview

**Gazebo** is the most popular open-source robotics simulator:

- **Physics Engines**: ODE, Bullet, DART, Simbody
- **Sensor Simulation**: Cameras, LiDAR, IMU, GPS
- **ROS2 Integration**: Native support via gazebo_ros
- **World Building**: GUI tools for environment creation

### Gazebo Architecture

```text
┌─────────────────────────────────────────────────────────┐
│                       Gazebo                            │
├─────────────────────────────────────────────────────────┤
│  ┌───────────────┐  ┌───────────────┐  ┌─────────────┐│
│  │  Physics      │  │   Rendering   │  │   Sensors   ││
│  │  Engine       │  │   Engine      │  │   Plugins   ││
│  └───────────────┘  └───────────────┘  └─────────────┘│
├─────────────────────────────────────────────────────────┤
│  ┌───────────────┐  ┌───────────────┐  ┌─────────────┐│
│  │  World        │  │   Model       │  │   gazebo_   ││
│  │  Files        │  │   Files       │  │   ros       ││
│  └───────────────┘  └───────────────┘  └─────────────┘│
└─────────────────────────────────────────────────────────┘
```

## Setting Up Gazebo

### Installation

```bash
# Install Gazebo for ROS2 Humble
sudo apt install ros-humble-gazebo-ros-pkgs

# Verify installation
gazebo --version
```

### Running Gazebo with ROS2

```bash
# Launch empty world
ros2 launch gazebo_ros gazebo.launch.py

# Launch with specific world
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/world.sdf
```

### Gazebo GUI

The Gazebo interface provides:

- **3D View**: Visualize the simulation world
- **Model Panel**: Insert objects and robots
- **World Panel**: Adjust physics and lighting
- **Timeline**: Control simulation time

## Basic World Creation

Create a simple world with obstacles:

```xml title="simple_world.sdf"
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_world">
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Box obstacle -->
    <model name="box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Module Structure

This module covers:

1. **[Gazebo Basics](./gazebo-basics)** - World creation and physics
2. **[URDF Robots](./urdf-robots)** - Robot description format

## Summary

- **Simulation** is essential for safe, fast robotics development
- **Gazebo** provides physics, sensors, and ROS2 integration
- **World files** define the simulation environment
- **The sim-to-real gap** must be addressed for deployment

**Next**: [Gazebo Basics](./gazebo-basics) - Deep dive into Gazebo simulation.
