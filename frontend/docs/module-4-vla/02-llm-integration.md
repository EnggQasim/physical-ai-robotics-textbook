---
sidebar_position: 3
title: "LLM Integration for Robotics"
description: "Connecting large language models to robot control systems"
---

# LLM Integration for Robotics

Large Language Models (LLMs) enable robots to understand complex instructions, reason about tasks, and generate appropriate actions. This section covers integrating LLMs with robotic systems.

## Learning Objectives

- Connect LLMs to robot control pipelines
- Implement task planning with language models
- Handle multi-step reasoning for complex tasks
- Build safe and reliable LLM-robot interfaces

## LLM-Robot Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                  LLM-Robot Integration                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  User Input ──► ┌─────────────────┐                         │
│                 │   LLM Engine    │                         │
│  Robot State ──►│  (GPT-4/Claude) │                         │
│                 │                 │──► Task Plan            │
│  Environment ──►│   + Prompting   │                         │
│                 └─────────────────┘                         │
│                         │                                    │
│                         ▼                                    │
│                 ┌─────────────────┐                         │
│                 │  Action Parser  │──► Robot Commands       │
│                 └─────────────────┘                         │
│                         │                                    │
│                         ▼                                    │
│                 ┌─────────────────┐                         │
│                 │ Safety Checker  │──► Verified Commands    │
│                 └─────────────────┘                         │
│                         │                                    │
│                         ▼                                    │
│                 ┌─────────────────┐                         │
│                 │ Robot Executor  │──► Physical Actions     │
│                 └─────────────────┘                         │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Setting Up LLM Integration

### Using OpenAI API

```python title="llm_client.py"
#!/usr/bin/env python3
from openai import OpenAI
from dataclasses import dataclass
from typing import List, Dict, Any
import json

@dataclass
class RobotAction:
    """Structured robot action."""
    action_type: str
    parameters: Dict[str, Any]
    description: str

class RobotLLMClient:
    """LLM client for robot task planning."""

    def __init__(self, api_key: str, model: str = "gpt-4"):
        self.client = OpenAI(api_key=api_key)
        self.model = model

        # System prompt for robotics
        self.system_prompt = """You are a robot control assistant. You help convert natural language commands into structured robot actions.

Available actions:
- MOVE_TO: Move robot to a location. Parameters: x, y, theta
- PICK: Pick up an object. Parameters: object_name
- PLACE: Place held object. Parameters: location_name
- LOOK_AT: Point camera at target. Parameters: target_name
- WAIT: Wait for duration. Parameters: seconds
- SPEAK: Say something. Parameters: text

Always respond with a JSON array of actions. Example:
[
  {"action": "MOVE_TO", "params": {"x": 1.0, "y": 2.0, "theta": 0}, "description": "Move to pickup location"},
  {"action": "PICK", "params": {"object_name": "cup"}, "description": "Pick up the cup"}
]

Consider safety: avoid collisions, check reachability, and confirm before dangerous actions."""

    def plan_task(self, command: str, context: Dict[str, Any] = None) -> List[RobotAction]:
        """
        Generate action plan from natural language command.

        Args:
            command: Natural language task description
            context: Optional context (robot state, environment info)

        Returns:
            List of RobotAction objects
        """
        # Build context message
        context_str = ""
        if context:
            context_str = f"\n\nCurrent context:\n{json.dumps(context, indent=2)}"

        # Query LLM
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": f"Task: {command}{context_str}"}
            ],
            response_format={"type": "json_object"},
            temperature=0.1  # Low temperature for consistency
        )

        # Parse response
        result = json.loads(response.choices[0].message.content)
        actions = result.get("actions", [])

        return [
            RobotAction(
                action_type=a["action"],
                parameters=a["params"],
                description=a["description"]
            )
            for a in actions
        ]

# Example usage
if __name__ == "__main__":
    client = RobotLLMClient(api_key="your-api-key")

    context = {
        "robot_position": {"x": 0, "y": 0},
        "objects_visible": ["cup", "plate", "fork"],
        "gripper_state": "empty"
    }

    actions = client.plan_task(
        "Pick up the cup and bring it to the table",
        context=context
    )

    for action in actions:
        print(f"{action.action_type}: {action.description}")
        print(f"  Parameters: {action.parameters}")
```

### Using Local LLMs (Ollama)

```python title="local_llm.py"
#!/usr/bin/env python3
import requests
import json
from typing import List, Dict, Any

class LocalLLMClient:
    """Client for local LLMs using Ollama."""

    def __init__(self, model: str = "llama2", host: str = "http://localhost:11434"):
        self.model = model
        self.host = host
        self.endpoint = f"{host}/api/generate"

    def generate(self, prompt: str, system: str = None) -> str:
        """Generate response from local LLM."""
        payload = {
            "model": self.model,
            "prompt": prompt,
            "stream": False
        }

        if system:
            payload["system"] = system

        response = requests.post(self.endpoint, json=payload)
        return response.json()["response"]

    def plan_robot_task(self, command: str, context: Dict = None) -> List[Dict]:
        """Plan robot task using local LLM."""
        system_prompt = """You are a robot planner. Output JSON array of actions.
Actions: MOVE_TO(x,y), PICK(object), PLACE(location), WAIT(seconds)
Only output valid JSON, no explanation."""

        prompt = f"Task: {command}"
        if context:
            prompt += f"\nContext: {json.dumps(context)}"
        prompt += "\nOutput actions as JSON array:"

        response = self.generate(prompt, system_prompt)

        try:
            # Try to extract JSON from response
            import re
            json_match = re.search(r'\[.*\]', response, re.DOTALL)
            if json_match:
                return json.loads(json_match.group())
        except json.JSONDecodeError:
            pass

        return []
```

## ROS2 LLM Integration Node

```python title="llm_planner_node.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import json
from openai import OpenAI

class LLMPlannerNode(Node):
    """ROS2 node for LLM-based task planning."""

    def __init__(self):
        super().__init__('llm_planner_node')

        # Parameters
        self.declare_parameter('api_key', '')
        self.declare_parameter('model', 'gpt-4')

        api_key = self.get_parameter('api_key').value
        model = self.get_parameter('model').value

        # Initialize LLM client
        self.client = OpenAI(api_key=api_key)
        self.model = model

        # Robot state
        self.robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'gripper': 'open',
            'battery': 100
        }

        # Publishers
        self.plan_pub = self.create_publisher(String, 'task_plan', 10)
        self.status_pub = self.create_publisher(String, 'planner_status', 10)

        # Subscribers
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.command_callback, 10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped, 'robot_pose', self.pose_callback, 10
        )

        # System prompt
        self.system_prompt = self._build_system_prompt()

        self.get_logger().info('LLM Planner node initialized')

    def _build_system_prompt(self) -> str:
        """Build system prompt for robot planning."""
        return """You are an intelligent robot planner. Convert natural language commands into executable robot action sequences.

## Available Robot Actions
1. NAVIGATE(x, y, theta) - Move to position
2. PICK(object_name) - Grasp an object
3. PLACE(location) - Place held object
4. LOOK(direction) - Turn camera
5. WAIT(seconds) - Pause execution
6. SAY(text) - Speak to user

## Response Format
Respond with JSON:
{
  "reasoning": "Brief explanation of plan",
  "actions": [
    {"action": "ACTION_NAME", "params": {...}, "step": 1}
  ],
  "estimated_time": seconds,
  "risks": ["potential issues"]
}

## Safety Rules
- Never navigate through obstacles
- Verify object is graspable before picking
- Confirm dangerous actions with user
- Stay within workspace boundaries"""

    def pose_callback(self, msg: PoseStamped):
        """Update robot position from pose message."""
        self.robot_state['position'] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }

    def command_callback(self, msg: String):
        """Process incoming command and generate plan."""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Publish status
        status_msg = String()
        status_msg.data = "planning"
        self.status_pub.publish(status_msg)

        try:
            # Generate plan with LLM
            plan = self._generate_plan(command)

            # Publish plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

            self.get_logger().info(f'Generated plan with {len(plan.get("actions", []))} actions')

            # Update status
            status_msg.data = "ready"
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Planning failed: {e}')
            status_msg.data = "error"
            self.status_pub.publish(status_msg)

    def _generate_plan(self, command: str) -> Dict:
        """Generate task plan using LLM."""
        context = {
            "robot_state": self.robot_state,
            "timestamp": self.get_clock().now().to_msg()
        }

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": f"Command: {command}\n\nContext: {json.dumps(context)}"}
            ],
            response_format={"type": "json_object"},
            temperature=0.1
        )

        return json.loads(response.choices[0].message.content)

def main():
    rclpy.init()
    node = LLMPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Task Decomposition

Break complex tasks into subtasks:

```python title="task_decomposer.py"
#!/usr/bin/env python3
from typing import List, Dict
from dataclasses import dataclass
from openai import OpenAI

@dataclass
class SubTask:
    """A decomposed subtask."""
    id: int
    description: str
    prerequisites: List[int]
    actions: List[Dict]
    verification: str

class TaskDecomposer:
    """Decompose complex tasks into manageable subtasks."""

    def __init__(self, client: OpenAI, model: str = "gpt-4"):
        self.client = client
        self.model = model

    def decompose(self, task: str, context: Dict = None) -> List[SubTask]:
        """
        Decompose a complex task into subtasks.

        Args:
            task: High-level task description
            context: Environment and robot context

        Returns:
            List of SubTask objects
        """
        prompt = f"""Decompose this robot task into subtasks:

Task: {task}

For each subtask, provide:
1. Description
2. Prerequisites (which subtasks must complete first)
3. Specific robot actions
4. How to verify completion

Output as JSON:
{{
  "subtasks": [
    {{
      "id": 1,
      "description": "...",
      "prerequisites": [],
      "actions": [...],
      "verification": "..."
    }}
  ]
}}"""

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": "You are a robot task planner."},
                {"role": "user", "content": prompt}
            ],
            response_format={"type": "json_object"}
        )

        result = json.loads(response.choices[0].message.content)

        return [
            SubTask(
                id=st["id"],
                description=st["description"],
                prerequisites=st["prerequisites"],
                actions=st["actions"],
                verification=st["verification"]
            )
            for st in result["subtasks"]
        ]

# Example: Decompose "Make coffee" task
if __name__ == "__main__":
    import os

    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    decomposer = TaskDecomposer(client)

    subtasks = decomposer.decompose("Make a cup of coffee")

    print("Task Decomposition:")
    for st in subtasks:
        print(f"\n{st.id}. {st.description}")
        print(f"   Prerequisites: {st.prerequisites}")
        print(f"   Actions: {st.actions}")
        print(f"   Verification: {st.verification}")
```

## Safety Validation

Validate LLM outputs before execution:

```python title="safety_validator.py"
#!/usr/bin/env python3
from typing import List, Dict, Tuple
from dataclasses import dataclass

@dataclass
class ValidationResult:
    """Result of safety validation."""
    is_safe: bool
    issues: List[str]
    modified_plan: Dict = None

class SafetyValidator:
    """Validate robot action plans for safety."""

    def __init__(self):
        # Define workspace boundaries
        self.workspace = {
            'x_min': -5.0, 'x_max': 5.0,
            'y_min': -5.0, 'y_max': 5.0,
            'z_min': 0.0, 'z_max': 2.0
        }

        # Maximum velocities
        self.max_linear_vel = 1.0  # m/s
        self.max_angular_vel = 1.5  # rad/s

        # Forbidden zones (e.g., near humans)
        self.forbidden_zones = []

        # Dangerous actions requiring confirmation
        self.dangerous_actions = ['DROP', 'THROW', 'PUSH']

    def validate(self, plan: Dict) -> ValidationResult:
        """
        Validate a robot action plan.

        Args:
            plan: Action plan from LLM

        Returns:
            ValidationResult with safety assessment
        """
        issues = []

        actions = plan.get('actions', [])

        for i, action in enumerate(actions):
            action_issues = self._validate_action(action, i)
            issues.extend(action_issues)

        # Check for dangerous actions
        dangerous = self._check_dangerous_actions(actions)
        issues.extend(dangerous)

        return ValidationResult(
            is_safe=len(issues) == 0,
            issues=issues
        )

    def _validate_action(self, action: Dict, index: int) -> List[str]:
        """Validate a single action."""
        issues = []
        action_type = action.get('action', '')
        params = action.get('params', {})

        # Check navigation targets
        if action_type == 'NAVIGATE':
            x, y = params.get('x', 0), params.get('y', 0)

            if not self._in_workspace(x, y):
                issues.append(
                    f"Action {index}: Navigation target ({x}, {y}) outside workspace"
                )

            if self._in_forbidden_zone(x, y):
                issues.append(
                    f"Action {index}: Navigation target ({x}, {y}) in forbidden zone"
                )

        # Check velocity limits
        if action_type == 'MOVE':
            vel = params.get('velocity', 0)
            if vel > self.max_linear_vel:
                issues.append(
                    f"Action {index}: Velocity {vel} exceeds limit {self.max_linear_vel}"
                )

        return issues

    def _in_workspace(self, x: float, y: float) -> bool:
        """Check if position is within workspace."""
        return (
            self.workspace['x_min'] <= x <= self.workspace['x_max'] and
            self.workspace['y_min'] <= y <= self.workspace['y_max']
        )

    def _in_forbidden_zone(self, x: float, y: float) -> bool:
        """Check if position is in a forbidden zone."""
        for zone in self.forbidden_zones:
            if (zone['x_min'] <= x <= zone['x_max'] and
                zone['y_min'] <= y <= zone['y_max']):
                return True
        return False

    def _check_dangerous_actions(self, actions: List[Dict]) -> List[str]:
        """Check for dangerous actions."""
        issues = []

        for i, action in enumerate(actions):
            if action.get('action') in self.dangerous_actions:
                issues.append(
                    f"Action {i}: '{action['action']}' requires user confirmation"
                )

        return issues
```

## Conversational Robot Interface

Enable multi-turn conversations:

```python title="conversation_manager.py"
#!/usr/bin/env python3
from typing import List, Dict
from dataclasses import dataclass, field
from openai import OpenAI
import json

@dataclass
class ConversationTurn:
    """A single turn in the conversation."""
    role: str
    content: str
    timestamp: float = 0.0

class ConversationManager:
    """Manage multi-turn conversations with the robot."""

    def __init__(self, client: OpenAI, model: str = "gpt-4"):
        self.client = client
        self.model = model
        self.history: List[ConversationTurn] = []
        self.max_history = 20  # Keep last 20 turns

        self.system_prompt = """You are a helpful robot assistant. You can:
1. Execute physical tasks (moving, picking, placing)
2. Answer questions about the environment
3. Clarify ambiguous commands
4. Explain what you're doing

When you need to execute actions, respond with:
{
  "response": "Natural language response to user",
  "actions": [list of robot actions],
  "clarification_needed": false
}

When you need clarification:
{
  "response": "Question for the user",
  "actions": [],
  "clarification_needed": true
}"""

    def process_input(self, user_input: str, context: Dict = None) -> Dict:
        """
        Process user input and generate response.

        Args:
            user_input: User's message
            context: Current robot/environment context

        Returns:
            Response dict with text and optional actions
        """
        # Add user message to history
        self.history.append(ConversationTurn(
            role="user",
            content=user_input
        ))

        # Build messages for API
        messages = [{"role": "system", "content": self.system_prompt}]

        # Add context
        if context:
            messages.append({
                "role": "system",
                "content": f"Current context: {json.dumps(context)}"
            })

        # Add conversation history
        for turn in self.history[-self.max_history:]:
            messages.append({
                "role": turn.role,
                "content": turn.content
            })

        # Query LLM
        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            response_format={"type": "json_object"}
        )

        result = json.loads(response.choices[0].message.content)

        # Add assistant response to history
        self.history.append(ConversationTurn(
            role="assistant",
            content=result.get("response", "")
        ))

        return result

    def reset(self):
        """Clear conversation history."""
        self.history = []

# Example conversation
if __name__ == "__main__":
    import os

    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    manager = ConversationManager(client)

    context = {
        "objects": ["cup", "plate", "fork"],
        "robot_location": "kitchen"
    }

    # Multi-turn conversation
    inputs = [
        "What can you see?",
        "Pick up the cup",
        "Where should I put it?",
        "Put it on the table please"
    ]

    for user_input in inputs:
        print(f"\nUser: {user_input}")
        response = manager.process_input(user_input, context)
        print(f"Robot: {response['response']}")
        if response.get('actions'):
            print(f"Actions: {response['actions']}")
```

## Summary

- **LLM integration** enables natural language robot control
- **Task decomposition** breaks complex tasks into executable steps
- **Safety validation** prevents dangerous actions
- **Conversational interfaces** allow multi-turn interaction
- **Local LLMs** provide privacy and offline operation

**Congratulations!** You've completed the Physical AI & Humanoid Robotics textbook. You now have the foundation to build intelligent robots using ROS2, simulation, NVIDIA Isaac, and cutting-edge AI models.
