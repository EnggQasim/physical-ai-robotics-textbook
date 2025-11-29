---
sidebar_position: 2
title: "Nodes and Topics"
description: "Understanding ROS2 nodes and the publish-subscribe pattern with topics"
---

# ROS2 Nodes and Topics

In this section, you'll learn about ROS2's fundamental building blocks: **nodes** and **topics**. These concepts form the backbone of any ROS2 application.

## Learning Objectives

- Create ROS2 nodes in Python
- Implement publishers and subscribers
- Understand message types and QoS settings
- Build a complete pub-sub system

## What are Nodes?

A **node** is a single-purpose process in ROS2. Each node should do one thing well, following the Unix philosophy.

### Node Characteristics

- **Single Responsibility**: One task per node
- **Modular**: Easy to replace or upgrade
- **Reusable**: Can be used in different robot configurations
- **Debuggable**: Isolated failures, easier testing

### Example Node Architecture

A typical robot might have these nodes:

```text
┌─────────────────────────────────────────────────────────┐
│                     Robot System                         │
├─────────────────────────────────────────────────────────┤
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐   │
│  │ Camera  │  │ LiDAR   │  │ Motor   │  │ Battery │   │
│  │  Node   │  │  Node   │  │ Control │  │ Monitor │   │
│  └─────────┘  └─────────┘  └─────────┘  └─────────┘   │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐   │
│  │ Object  │  │  SLAM   │  │  Path   │  │ Safety  │   │
│  │Detector │  │  Node   │  │ Planner │  │  Node   │   │
│  └─────────┘  └─────────┘  └─────────┘  └─────────┘   │
└─────────────────────────────────────────────────────────┘
```

## Creating Your First Node

Let's create a simple ROS2 node in Python:

```python title="minimal_node.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    """A minimal ROS2 node that logs a message."""

    def __init__(self):
        # Initialize the node with a name
        super().__init__('minimal_node')

        # Log that the node has started
        self.get_logger().info('Minimal node has started!')

        # Create a timer that fires every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """Called every time the timer fires."""
        self.counter += 1
        self.get_logger().info(f'Timer fired {self.counter} times')

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create and spin the node
    node = MinimalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

Every ROS2 node follows this lifecycle:

1. **Initialize**: `rclpy.init()` - Set up the ROS2 context
2. **Create**: Instantiate your node class
3. **Spin**: `rclpy.spin()` - Process callbacks
4. **Shutdown**: `rclpy.shutdown()` - Clean up resources

## What are Topics?

**Topics** are named buses for message passing. They implement the **publish-subscribe** pattern:

- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- **Multiple-to-multiple**: Any number of publishers and subscribers

![ROS2 Publisher-Subscriber Pattern](/img/generated/ros2/pubsub-diagram.svg)
*Figure: ROS2 Publisher-Subscriber Communication Pattern - Publishers send data to a named topic, and subscribers receive data from that topic.*

### Topic Characteristics

- **Asynchronous**: Publishers don't wait for subscribers
- **Decoupled**: Publishers and subscribers don't know each other
- **Typed**: Each topic has a specific message type
- **Named**: Topics have unique string names (e.g., `/robot/velocity`)

## Understanding Messages

Messages define the structure of data sent over topics. ROS2 provides many standard messages:

### Common Message Types

| Package | Message | Description |
|---------|---------|-------------|
| std_msgs | String | Simple string |
| std_msgs | Int32, Float64 | Numeric types |
| geometry_msgs | Twist | Linear and angular velocity |
| geometry_msgs | Pose | Position and orientation |
| sensor_msgs | Image | Camera images |
| sensor_msgs | LaserScan | LiDAR data |

### Example: Twist Message

The `Twist` message is commonly used for robot velocity commands:

```python
from geometry_msgs.msg import Twist

# Create a velocity command
vel = Twist()
vel.linear.x = 0.5   # Forward velocity (m/s)
vel.linear.y = 0.0   # Lateral velocity
vel.linear.z = 0.0   # Vertical velocity
vel.angular.x = 0.0  # Roll rate
vel.angular.y = 0.0  # Pitch rate
vel.angular.z = 0.1  # Yaw rate (rad/s)
```

## Creating a Publisher

Let's create a node that publishes velocity commands:

```python title="velocity_publisher.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    """Publishes velocity commands to move a robot in a circle."""

    def __init__(self):
        super().__init__('velocity_publisher')

        # Create a publisher
        # Arguments: message type, topic name, queue size
        self.publisher = self.create_publisher(
            Twist,           # Message type
            'cmd_vel',       # Topic name
            10               # Queue size
        )

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.get_logger().info('Velocity publisher started')

    def publish_velocity(self):
        """Publish a velocity command."""
        msg = Twist()

        # Move forward at 0.5 m/s while rotating at 0.3 rad/s
        msg.linear.x = 0.5
        msg.angular.z = 0.3

        # Publish the message
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

## Creating a Subscriber

Now let's create a node that subscribes to velocity commands:

```python title="velocity_subscriber.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocitySubscriber(Node):
    """Subscribes to velocity commands and logs them."""

    def __init__(self):
        super().__init__('velocity_subscriber')

        # Create a subscription
        self.subscription = self.create_subscription(
            Twist,                    # Message type
            'cmd_vel',                # Topic name
            self.velocity_callback,   # Callback function
            10                        # Queue size
        )

        self.get_logger().info('Velocity subscriber started')

    def velocity_callback(self, msg: Twist):
        """Called when a velocity message is received."""
        self.get_logger().info(
            f'Received velocity: '
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

QoS settings control how messages are delivered:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Define custom QoS
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # Guaranteed delivery
    history=HistoryPolicy.KEEP_LAST,         # Keep last N messages
    depth=10                                  # Queue size
)

# Use QoS in publisher
self.publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile)
```

### Common QoS Presets

| Preset | Use Case |
|--------|----------|
| Sensor Data | Best effort, volatile (fast, may drop) |
| Services | Reliable, volatile (guaranteed but not stored) |
| Parameters | Reliable, transient local (stored for late subscribers) |

## Running the Example

To test your publisher and subscriber:

```bash
# Terminal 1: Run the publisher
python3 velocity_publisher.py

# Terminal 2: Run the subscriber
python3 velocity_subscriber.py

# Terminal 3: List topics and echo messages
ros2 topic list
ros2 topic echo /cmd_vel
```

## ROS2 CLI Tools

Useful commands for working with topics:

```bash
# List all active topics
ros2 topic list

# Show topic info
ros2 topic info /cmd_vel

# Echo messages (subscribe from CLI)
ros2 topic echo /cmd_vel

# Publish from CLI
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# Check publishing rate
ros2 topic hz /cmd_vel

# Show message type definition
ros2 interface show geometry_msgs/msg/Twist
```

## Exercises

1. **Exercise 1**: Modify the publisher to publish a `String` message with your name
2. **Exercise 2**: Create a subscriber that counts how many messages it receives
3. **Exercise 3**: Build a system where one node publishes random numbers and another calculates the running average

## Summary

In this section, you learned:

- **Nodes** are single-purpose processes that communicate via ROS2
- **Topics** enable publish-subscribe communication
- **Messages** define the structure of data exchanged
- **QoS** settings control delivery guarantees
- **CLI tools** help debug and inspect the system

**Next**: [Services and Actions](./services-actions) - Request-response patterns in ROS2.
