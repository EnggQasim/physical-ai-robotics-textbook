---
sidebar_position: 1
title: "ROS2 Overview"
description: "Introduction to ROS2 - The Robot Operating System 2"
---

# ROS2 Fundamentals

Welcome to Module 1 of the Physical AI Textbook. In this module, you'll master **Robot Operating System 2 (ROS2)**, the foundation of modern robotics software development.

## Learning Objectives

By the end of this module, you will be able to:

- Understand the ROS2 architecture and design principles
- Create and manage ROS2 nodes in Python
- Implement publisher-subscriber communication with topics
- Build request-response patterns with services
- Handle long-running tasks with actions
- Structure ROS2 packages for your projects

## What is ROS2?

**ROS2** (Robot Operating System 2) is not an operating system in the traditional sense. It's a **middleware framework** that provides:

1. **Communication Layer**: Tools for inter-process communication
2. **Hardware Abstraction**: Standard interfaces for sensors and actuators
3. **Device Drivers**: Pre-built drivers for common robotics hardware
4. **Libraries**: Algorithms for navigation, perception, manipulation
5. **Tools**: Visualization, debugging, simulation integration

### ROS vs ROS2

ROS2 is a complete redesign of the original ROS, addressing key limitations:

| Aspect | ROS1 | ROS2 |
|--------|------|------|
| Communication | Custom (TCPROS) | DDS Standard |
| Real-time | Not supported | Supported |
| Security | None built-in | DDS-Security |
| Multi-robot | Difficult | Native support |
| Platforms | Linux only | Linux, Windows, macOS |
| Python | Python 2 | Python 3 |

### Why ROS2 Matters

ROS2 has become the industry standard because:

- **Open Source**: Free to use, modify, and distribute
- **Large Ecosystem**: Thousands of packages and active community
- **Industry Adoption**: Used by major robotics companies
- **Modern Design**: Built for production, not just research

## Core Concepts Overview

ROS2 is built around several key concepts that we'll explore in detail:

### 1. Nodes

**Nodes** are the fundamental building blocks of ROS2 applications. Each node is a process that performs a specific task.

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        self.get_logger().info('Node started!')
```

Think of nodes as microservices - each handles one responsibility.

### 2. Topics

**Topics** enable publish-subscribe communication. Publishers send messages to a topic, and subscribers receive them.

```text
┌──────────────┐      /robot/velocity      ┌──────────────┐
│   Joystick   │ ─────────────────────────► │    Robot     │
│   Node       │      (Publisher)           │   Controller │
└──────────────┘                            │   (Subscriber)│
                                            └──────────────┘
```

### 3. Services

**Services** provide request-response communication. A client sends a request, and a server responds.

```text
┌──────────────┐                            ┌──────────────┐
│    Client    │ ──── Request ────────────► │    Server    │
│              │ ◄─── Response ───────────  │              │
└──────────────┘                            └──────────────┘
```

### 4. Actions

**Actions** handle long-running tasks with feedback. They combine services with topics for progress updates.

```text
┌──────────────┐      Goal       ┌──────────────┐
│    Client    │ ───────────────►│    Server    │
│              │ ◄── Feedback ─── │              │
│              │ ◄── Result ───── │              │
└──────────────┘                  └──────────────┘
```

## ROS2 Architecture

The ROS2 architecture consists of several layers:

```text
┌─────────────────────────────────────────────────────────┐
│                   Your Application                       │
│              (Nodes, Packages, Launch Files)            │
├─────────────────────────────────────────────────────────┤
│                   ROS2 Client Libraries                  │
│              (rclpy, rclcpp, rclnodejs)                 │
├─────────────────────────────────────────────────────────┤
│                      ROS2 Core (rcl)                    │
│              (Node, Publisher, Subscriber APIs)         │
├─────────────────────────────────────────────────────────┤
│                    ROS Middleware (rmw)                  │
│              (Abstraction over DDS)                      │
├─────────────────────────────────────────────────────────┤
│                  DDS Implementation                      │
│        (Fast-DDS, Cyclone DDS, RTI Connext)             │
└─────────────────────────────────────────────────────────┘
```

### Data Distribution Service (DDS)

ROS2 uses **DDS** (Data Distribution Service) as its communication layer:

- **Decentralized**: No central master (unlike ROS1's roscore)
- **Automatic Discovery**: Nodes find each other automatically
- **Quality of Service (QoS)**: Configurable reliability, durability, deadline
- **Real-time Capable**: Deterministic message delivery

## Setting Up Your Environment

### Prerequisites

Before starting, ensure you have:

1. **Ubuntu 22.04** (recommended) or Windows/macOS
2. **Python 3.10+**
3. **ROS2 Humble** or newer

### Installation

On Ubuntu 22.04, install ROS2 Humble:

```bash
# Add ROS2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source the setup script
source /opt/ros/humble/setup.bash
```

### Verify Installation

Test your installation:

```bash
# Check ROS2 version
ros2 --version

# Run a demo
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
```

You should see messages being published and received.

## Module Structure

This module is organized into the following sections:

1. **[Nodes and Topics](./nodes-topics)** - Publisher-subscriber communication
2. **[Services and Actions](./services-actions)** - Request-response and long-running tasks
3. **[Python Integration](./python-rclpy)** - Building ROS2 applications with rclpy

Each section includes:
- Conceptual explanations
- Working code examples
- Hands-on exercises

## Summary

In this overview, you learned:

- ROS2 is a middleware framework for robotics development
- It provides communication, abstraction, and tools
- Core concepts include nodes, topics, services, and actions
- ROS2 uses DDS for decentralized, real-time communication

**Next**: [Nodes and Topics](./nodes-topics) - Dive into the publisher-subscriber pattern.
