---
sidebar_position: 3
title: "Services and Actions"
description: "Request-response patterns with ROS2 services and long-running tasks with actions"
---

# ROS2 Services and Actions

While topics are great for continuous data streams, many robotics tasks require **request-response** communication or **long-running operations with feedback**. That's where **services** and **actions** come in.

## Learning Objectives

- Understand when to use services vs topics vs actions
- Create ROS2 service servers and clients
- Implement action servers for long-running tasks
- Handle feedback and cancellation in actions

## When to Use What?

| Pattern | Use Case | Example |
|---------|----------|---------|
| **Topics** | Continuous data streams | Sensor data, velocity commands |
| **Services** | Quick request-response | Get robot state, set parameters |
| **Actions** | Long-running tasks with feedback | Navigation, arm motion |

## ROS2 Services

Services provide **synchronous request-response** communication:

![ROS2 Service-Client Communication](/img/generated/ros2/service-diagram.svg)
*Figure: ROS2 Service-Client Pattern - A client sends a request to a service server and waits for the response.*

### Service Characteristics

- **Synchronous**: Client waits for response
- **One-to-one**: One request, one response
- **Typed**: Defined by `.srv` files
- **Short-lived**: For quick operations

### Service Definition

Services are defined in `.srv` files with request and response separated by `---`:

```text title="AddTwoInts.srv"
# Request
int64 a
int64 b
---
# Response
int64 sum
```

### Creating a Service Server

```python title="add_two_ints_server.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    """A service server that adds two integers."""

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create the service
        self.srv = self.create_service(
            AddTwoInts,              # Service type
            'add_two_ints',          # Service name
            self.add_callback        # Callback function
        )

        self.get_logger().info('Add Two Ints service is ready')

    def add_callback(self, request, response):
        """Handle incoming service requests."""
        response.sum = request.a + request.b

        self.get_logger().info(
            f'Received: {request.a} + {request.b} = {response.sum}'
        )

        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()

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

### Creating a Service Client

```python title="add_two_ints_client.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    """A service client that requests addition of two integers."""

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create the client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.get_logger().info('Service is available')

    def send_request(self, a: int, b: int):
        """Send a request and wait for response."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call the service asynchronously
        future = self.client.call_async(request)

        # Wait for the result
        rclpy.spin_until_future_complete(self, future)

        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()

    # Send a request
    response = node.send_request(5, 3)
    node.get_logger().info(f'Result: {response.sum}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service CLI Tools

```bash
# List all services
ros2 service list

# Check service type
ros2 service type /add_two_ints

# Call service from CLI
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

## ROS2 Actions

Actions are designed for **long-running tasks** that need:

- **Goal submission**: Start a task
- **Feedback**: Progress updates during execution
- **Result**: Final outcome when complete
- **Cancellation**: Ability to stop the task

### Action Architecture

```text
┌──────────────┐                            ┌──────────────┐
│    Client    │ ───── Goal ───────────────►│    Server    │
│              │ ◄──── Accept/Reject ─────  │              │
│              │                            │              │
│              │ ◄──── Feedback ──────────  │  (executing) │
│              │ ◄──── Feedback ──────────  │              │
│              │ ◄──── Feedback ──────────  │              │
│              │                            │              │
│              │ ◄──── Result ────────────  │  (complete)  │
└──────────────┘                            └──────────────┘
```

### Action Definition

Actions are defined in `.action` files with three parts:

```text title="Fibonacci.action"
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

### Action Server Example

```python title="fibonacci_action_server.py"
#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    """An action server that computes Fibonacci sequences."""

    def __init__(self):
        super().__init__('fibonacci_action_server')

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.get_logger().info('Fibonacci action server is ready')

    def execute_callback(self, goal_handle):
        """Execute the Fibonacci computation."""
        self.get_logger().info(f'Computing Fibonacci({goal_handle.request.order})')

        # Initialize sequence
        sequence = [0, 1]

        # Build Fibonacci sequence with feedback
        for i in range(1, goal_handle.request.order):
            # Check if the goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Compute next number
            sequence.append(sequence[i] + sequence[i - 1])

            # Send feedback
            feedback = Fibonacci.Feedback()
            feedback.partial_sequence = sequence
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f'Feedback: {sequence}')

            # Simulate work
            time.sleep(0.5)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()

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

### Action Client Example

```python title="fibonacci_action_client.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    """An action client that requests Fibonacci computation."""

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self.get_logger().info('Fibonacci action client created')

    def send_goal(self, order: int):
        """Send a goal to the action server."""
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: order={order}')

        # Send goal with feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when the goal is accepted or rejected."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Called when feedback is received."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        """Called when the result is available."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = FibonacciActionClient()
    client.send_goal(10)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

### Action CLI Tools

```bash
# List all actions
ros2 action list

# Show action info
ros2 action info /fibonacci

# Send goal from CLI
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}"

# Send goal with feedback
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}" --feedback
```

## Real-World Example: Navigation Action

In robotics, navigation is a perfect use case for actions:

```python title="navigate_to_pose.py"
#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class NavigationClient:
    """Client for sending navigation goals."""

    def navigate_to(self, x: float, y: float, theta: float):
        """Navigate the robot to a pose."""
        goal = NavigateToPose.Goal()

        # Set target pose
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = sin(theta / 2)
        goal.pose.pose.orientation.w = cos(theta / 2)

        # Send goal and wait for result
        future = self.client.send_goal_async(goal)
        # ... handle feedback and result
```

## Comparison Summary

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| Pattern | Pub-Sub | Req-Resp | Goal-Feedback-Result |
| Timing | Async | Sync | Async with updates |
| Duration | Continuous | Quick | Long-running |
| Feedback | N/A | N/A | Yes |
| Cancellation | N/A | N/A | Yes |

## Exercises

1. **Exercise 1**: Create a service that converts Celsius to Fahrenheit
2. **Exercise 2**: Build an action server that counts to a given number with 1-second delays
3. **Exercise 3**: Implement an action client that can cancel the goal after receiving 3 feedback messages

## Summary

In this section, you learned:

- **Services** provide synchronous request-response communication
- **Actions** handle long-running tasks with feedback and cancellation
- Service servers and clients exchange typed messages
- Action servers can send periodic feedback during execution
- CLI tools help test and debug services and actions

**Next**: [Python Integration with rclpy](./python-rclpy) - Deep dive into Python ROS2 development.
