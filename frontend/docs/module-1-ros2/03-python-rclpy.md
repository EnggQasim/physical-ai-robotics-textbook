---
sidebar_position: 4
title: "Python Integration (rclpy)"
description: "Building ROS2 applications with Python using the rclpy client library"
---

# Python Integration with rclpy

**rclpy** is the official Python client library for ROS2. It provides a Pythonic API for creating nodes, publishers, subscribers, services, actions, and more.

## Learning Objectives

- Master the rclpy API for ROS2 development
- Create well-structured ROS2 Python packages
- Handle parameters and launch files
- Build reusable robot components

## rclpy Architecture

The rclpy library wraps the ROS2 core (rcl) in Python:

```text
┌─────────────────────────────────────────┐
│          Your Python Code               │
├─────────────────────────────────────────┤
│              rclpy                       │
│  (Node, Publisher, Subscriber, etc.)    │
├─────────────────────────────────────────┤
│          rcl (C library)                │
├─────────────────────────────────────────┤
│          rmw (ROS Middleware)           │
├─────────────────────────────────────────┤
│              DDS                        │
└─────────────────────────────────────────┘
```

## Creating a ROS2 Python Package

### Package Structure

A typical ROS2 Python package structure:

```text
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py
│   ├── my_node.py
│   └── utils.py
├── resource/
│   └── my_robot_pkg
├── test/
│   └── test_my_node.py
├── launch/
│   └── robot.launch.py
├── config/
│   └── params.yaml
├── package.xml
├── setup.py
└── setup.cfg
```

### Creating a Package

```bash
# Create a new package
ros2 pkg create --build-type ament_python my_robot_pkg

# Or with dependencies
ros2 pkg create --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs \
  my_robot_pkg
```

### setup.py Configuration

```python title="setup.py"
from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='My robot package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'my_node = my_robot_pkg.my_node:main',
            'controller = my_robot_pkg.controller:main',
        ],
    },
)
```

## Advanced Node Patterns

### Node with Multiple Components

```python title="robot_controller.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

class RobotController(Node):
    """A complete robot controller with sensors and actuators."""

    def __init__(self):
        super().__init__('robot_controller')

        # Declare parameters
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('safety_distance', 0.5)

        # Get parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for laser scan
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Service to enable/disable the controller
        self.enable_srv = self.create_service(
            SetBool, 'enable_controller', self.enable_callback
        )

        # Timer for control loop (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # State variables
        self.enabled = True
        self.obstacle_detected = False
        self.min_distance = float('inf')

        self.get_logger().info('Robot controller initialized')

    def scan_callback(self, msg: LaserScan):
        """Process laser scan data."""
        # Find minimum distance in front of robot
        # Assuming front is in the middle of the scan
        mid = len(msg.ranges) // 2
        front_ranges = msg.ranges[mid-30:mid+30]

        valid_ranges = [r for r in front_ranges if r > msg.range_min]
        self.min_distance = min(valid_ranges) if valid_ranges else float('inf')

        self.obstacle_detected = self.min_distance < self.safety_distance

    def enable_callback(self, request, response):
        """Handle enable/disable service requests."""
        self.enabled = request.data
        response.success = True
        response.message = f'Controller {"enabled" if self.enabled else "disabled"}'

        self.get_logger().info(response.message)
        return response

    def control_loop(self):
        """Main control loop - runs at 10 Hz."""
        if not self.enabled:
            self.stop_robot()
            return

        if self.obstacle_detected:
            self.get_logger().warn(
                f'Obstacle at {self.min_distance:.2f}m - stopping!'
            )
            self.stop_robot()
            return

        # Normal operation - move forward
        cmd = Twist()
        cmd.linear.x = self.max_speed
        self.vel_pub.publish(cmd)

    def stop_robot(self):
        """Send zero velocity command."""
        cmd = Twist()
        self.vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Parameters

### Declaring and Using Parameters

```python title="parameter_example.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class ParameterNode(Node):
    """Demonstrates ROS2 parameter handling."""

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with descriptors
        self.declare_parameter(
            'robot_name',
            'default_robot',
            ParameterDescriptor(description='Name of the robot')
        )

        self.declare_parameter(
            'max_velocity',
            1.0,
            ParameterDescriptor(
                description='Maximum velocity in m/s',
                floating_point_range=[{
                    'from_value': 0.0,
                    'to_value': 5.0,
                    'step': 0.1
                }]
            )
        )

        # Get parameter values
        robot_name = self.get_parameter('robot_name').value
        max_vel = self.get_parameter('max_velocity').value

        self.get_logger().info(f'Robot: {robot_name}, Max Vel: {max_vel}')

        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Called when parameters are changed."""
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            self.get_logger().info(
                f'Parameter {param.name} changed to {param.value}'
            )

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Parameter File (YAML)

```yaml title="config/params.yaml"
robot_controller:
  ros__parameters:
    robot_name: "my_humanoid"
    max_speed: 1.5
    safety_distance: 0.3
    control_frequency: 20.0
    debug_mode: false
```

## Launch Files

Launch files start multiple nodes with configuration:

```python title="launch/robot.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot_pkg')

    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_01',
        description='Name of the robot'
    )

    # Path to parameter file
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # Define nodes
    controller_node = Node(
        package='my_robot_pkg',
        executable='controller',
        name='robot_controller',
        parameters=[
            params_file,
            {'robot_name': LaunchConfiguration('robot_name')}
        ],
        output='screen',
        emulate_tty=True
    )

    sensor_node = Node(
        package='my_robot_pkg',
        executable='sensor_processor',
        name='sensor_processor',
        remappings=[
            ('input_scan', '/robot/lidar'),
            ('output_obstacles', '/robot/obstacles')
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        controller_node,
        sensor_node,
    ])
```

### Running Launch Files

```bash
# Run launch file
ros2 launch my_robot_pkg robot.launch.py

# With arguments
ros2 launch my_robot_pkg robot.launch.py robot_name:=humanoid_02
```

## Executors and Callbacks

### Multi-threaded Execution

```python title="multi_threaded_node.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class MultiThreadedNode(Node):
    """Node with multi-threaded callback execution."""

    def __init__(self):
        super().__init__('multi_threaded_node')

        # Callback groups
        self.reentrant_group = ReentrantCallbackGroup()
        self.exclusive_group = MutuallyExclusiveCallbackGroup()

        # Timer with reentrant group (can run in parallel)
        self.timer1 = self.create_timer(
            0.1,
            self.fast_callback,
            callback_group=self.reentrant_group
        )

        # Subscriber with exclusive group (one at a time)
        self.sub = self.create_subscription(
            String,
            'topic',
            self.message_callback,
            10,
            callback_group=self.exclusive_group
        )

    def fast_callback(self):
        # This can run in parallel with other reentrant callbacks
        pass

    def message_callback(self, msg):
        # This runs exclusively, not in parallel with other exclusive callbacks
        pass

def main():
    rclpy.init()
    node = MultiThreadedNode()

    # Use multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
```

## Building and Running

### Build the Package

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select my_robot_pkg

# Source the workspace
source install/setup.bash
```

### Run Nodes

```bash
# Run a single node
ros2 run my_robot_pkg my_node

# Run with parameters
ros2 run my_robot_pkg my_node --ros-args -p max_speed:=2.0

# Run launch file
ros2 launch my_robot_pkg robot.launch.py
```

## Best Practices

### 1. Use Type Hints

```python
from geometry_msgs.msg import Twist
from typing import Optional

def process_velocity(self, cmd: Twist) -> Optional[Twist]:
    """Process and validate velocity command."""
    if cmd.linear.x > self.max_speed:
        return None
    return cmd
```

### 2. Handle Errors Gracefully

```python
try:
    result = self.client.call(request)
except Exception as e:
    self.get_logger().error(f'Service call failed: {e}')
    return None
```

### 3. Use Logging Appropriately

```python
self.get_logger().debug('Detailed debug info')    # Development
self.get_logger().info('Normal operation')         # Status updates
self.get_logger().warn('Potential problem')        # Warnings
self.get_logger().error('Something went wrong')    # Errors
self.get_logger().fatal('Critical failure')        # System failures
```

## Exercises

1. **Exercise 1**: Create a package with a node that subscribes to `cmd_vel` and logs the velocity magnitude
2. **Exercise 2**: Build a launch file that starts three nodes with different parameter configurations
3. **Exercise 3**: Implement a node with dynamic parameter reconfiguration for tuning robot behavior

## Summary

In this section, you learned:

- **rclpy** provides a Pythonic API for ROS2 development
- **Packages** organize code, configuration, and launch files
- **Parameters** allow runtime configuration and dynamic updates
- **Launch files** start multiple nodes with coordinated configuration
- **Executors** control callback threading and concurrency

You now have the foundation to build sophisticated ROS2 applications in Python!

**Next Module**: [Robot Simulation](/docs/module-2-simulation) - Bring your robots to life in simulation.
