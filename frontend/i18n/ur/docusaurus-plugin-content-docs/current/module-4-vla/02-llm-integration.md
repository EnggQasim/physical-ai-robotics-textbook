---
sidebar_position: 3
title: "روبوٹکس کے لیے LLM انٹیگریشن"
description: "بڑے لینگویج ماڈلز کو روبوٹ کنٹرول سسٹمز سے جوڑنا"
---

# روبوٹکس کے لیے LLM انٹیگریشن (LLM Integration for Robotics)

بڑے لینگویج ماڈلز (LLMs) روبوٹس کو پیچیدہ ہدایات سمجھنے، ٹاسکس کے بارے میں استدلال کرنے، اور مناسب ایکشنز پیدا کرنے کے قابل بناتے ہیں۔ اس سیکشن میں LLMs کو روبوٹک سسٹمز سے جوڑنا شامل ہے۔

## سیکھنے کے مقاصد (Learning Objectives)

- LLMs کو روبوٹ کنٹرول پائپ لائنز سے جوڑیں
- لینگویج ماڈلز سے ٹاسک پلاننگ نافذ کریں
- پیچیدہ ٹاسکس کے لیے ملٹی سٹیپ ریزننگ سنبھالیں
- محفوظ اور قابل اعتماد LLM-روبوٹ انٹرفیسز بنائیں

## LLM-روبوٹ آرکیٹیکچر

```text
┌─────────────────────────────────────────────────────────────┐
│                  LLM-روبوٹ انٹیگریشن                         │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  یوزر ان پٹ ──► ┌─────────────────┐                         │
│                 │   LLM انجن     │                         │
│  روبوٹ اسٹیٹ ──►│  (GPT-4/Claude) │                         │
│                 │                 │──► ٹاسک پلان            │
│  ماحول ──►      │   + پرامپٹنگ   │                         │
│                 └─────────────────┘                         │
│                         │                                    │
│                         ▼                                    │
│                 ┌─────────────────┐                         │
│                 │  ایکشن پارسر  │──► روبوٹ کمانڈز          │
│                 └─────────────────┘                         │
│                         │                                    │
│                         ▼                                    │
│                 ┌─────────────────┐                         │
│                 │ سیفٹی چیکر    │──► تصدیق شدہ کمانڈز     │
│                 └─────────────────┘                         │
│                         │                                    │
│                         ▼                                    │
│                 ┌─────────────────┐                         │
│                 │روبوٹ ایگزیکیوٹر│──► فزیکل ایکشنز        │
│                 └─────────────────┘                         │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## LLM انٹیگریشن سیٹ اپ کرنا

### OpenAI API استعمال کرنا

```python title="llm_client.py"
#!/usr/bin/env python3
from openai import OpenAI
from dataclasses import dataclass
from typing import List, Dict, Any
import json

@dataclass
class RobotAction:
    """ڈھانچے دار روبوٹ ایکشن۔"""
    action_type: str
    parameters: Dict[str, Any]
    description: str

class RobotLLMClient:
    """روبوٹ ٹاسک پلاننگ کے لیے LLM کلائنٹ۔"""

    def __init__(self, api_key: str, model: str = "gpt-4"):
        self.client = OpenAI(api_key=api_key)
        self.model = model

        # روبوٹکس کے لیے سسٹم پرامپٹ
        self.system_prompt = """آپ روبوٹ کنٹرول اسسٹنٹ ہیں۔ آپ قدرتی زبان کی کمانڈز کو ڈھانچے دار روبوٹ ایکشنز میں تبدیل کرنے میں مدد کرتے ہیں۔

دستیاب ایکشنز:
- MOVE_TO: روبوٹ کو کسی جگہ لے جائیں۔ پیرامیٹرز: x, y, theta
- PICK: کوئی چیز اٹھائیں۔ پیرامیٹرز: object_name
- PLACE: پکڑی ہوئی چیز رکھیں۔ پیرامیٹرز: location_name
- LOOK_AT: کیمرہ ٹارگٹ کی طرف کریں۔ پیرامیٹرز: target_name
- WAIT: وقفہ کریں۔ پیرامیٹرز: seconds
- SPEAK: کچھ بولیں۔ پیرامیٹرز: text

ہمیشہ ایکشنز کی JSON ارے سے جواب دیں۔ مثال:
[
  {"action": "MOVE_TO", "params": {"x": 1.0, "y": 2.0, "theta": 0}, "description": "پک اپ لوکیشن پر جائیں"},
  {"action": "PICK", "params": {"object_name": "cup"}, "description": "کپ اٹھائیں"}
]

سیفٹی پر غور کریں: ٹکراؤ سے بچیں، رسائی چیک کریں، اور خطرناک ایکشنز سے پہلے تصدیق کریں۔"""

    def plan_task(self, command: str, context: Dict[str, Any] = None) -> List[RobotAction]:
        """
        قدرتی زبان کمانڈ سے ایکشن پلان بنائیں۔

        Args:
            command: قدرتی زبان ٹاسک کی تفصیل
            context: اختیاری سیاق (روبوٹ اسٹیٹ، ماحول کی معلومات)

        Returns:
            RobotAction آبجیکٹس کی فہرست
        """
        # سیاق پیغام بنائیں
        context_str = ""
        if context:
            context_str = f"\n\nموجودہ سیاق:\n{json.dumps(context, indent=2)}"

        # LLM سے پوچھیں
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": f"ٹاسک: {command}{context_str}"}
            ],
            response_format={"type": "json_object"},
            temperature=0.1  # مستقل مزاجی کے لیے کم ٹمپریچر
        )

        # جواب پارس کریں
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

# مثال کا استعمال
if __name__ == "__main__":
    client = RobotLLMClient(api_key="your-api-key")

    context = {
        "robot_position": {"x": 0, "y": 0},
        "objects_visible": ["cup", "plate", "fork"],
        "gripper_state": "empty"
    }

    actions = client.plan_task(
        "کپ اٹھاؤ اور میز پر لے آؤ",
        context=context
    )

    for action in actions:
        print(f"{action.action_type}: {action.description}")
        print(f"  پیرامیٹرز: {action.parameters}")
```

### لوکل LLMs استعمال کرنا (Ollama)

```python title="local_llm.py"
#!/usr/bin/env python3
import requests
import json
from typing import List, Dict, Any

class LocalLLMClient:
    """Ollama استعمال کرتے ہوئے لوکل LLMs کے لیے کلائنٹ۔"""

    def __init__(self, model: str = "llama2", host: str = "http://localhost:11434"):
        self.model = model
        self.host = host
        self.endpoint = f"{host}/api/generate"

    def generate(self, prompt: str, system: str = None) -> str:
        """لوکل LLM سے جواب بنائیں۔"""
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
        """لوکل LLM استعمال کرتے ہوئے روبوٹ ٹاسک کی منصوبہ بندی کریں۔"""
        system_prompt = """آپ روبوٹ پلانر ہیں۔ ایکشنز کی JSON ارے آؤٹ پٹ کریں۔
ایکشنز: MOVE_TO(x,y), PICK(object), PLACE(location), WAIT(seconds)
صرف درست JSON آؤٹ پٹ کریں، کوئی وضاحت نہیں۔"""

        prompt = f"ٹاسک: {command}"
        if context:
            prompt += f"\nسیاق: {json.dumps(context)}"
        prompt += "\nایکشنز کو JSON ارے کے طور پر آؤٹ پٹ کریں:"

        response = self.generate(prompt, system_prompt)

        try:
            # جواب سے JSON نکالنے کی کوشش کریں
            import re
            json_match = re.search(r'\[.*\]', response, re.DOTALL)
            if json_match:
                return json.loads(json_match.group())
        except json.JSONDecodeError:
            pass

        return []
```

## ROS2 LLM انٹیگریشن نوڈ

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
    """LLM بیسڈ ٹاسک پلاننگ کے لیے ROS2 نوڈ۔"""

    def __init__(self):
        super().__init__('llm_planner_node')

        # پیرامیٹرز
        self.declare_parameter('api_key', '')
        self.declare_parameter('model', 'gpt-4')

        api_key = self.get_parameter('api_key').value
        model = self.get_parameter('model').value

        # LLM کلائنٹ شروع کریں
        self.client = OpenAI(api_key=api_key)
        self.model = model

        # روبوٹ اسٹیٹ
        self.robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'gripper': 'open',
            'battery': 100
        }

        # پبلشرز
        self.plan_pub = self.create_publisher(String, 'task_plan', 10)
        self.status_pub = self.create_publisher(String, 'planner_status', 10)

        # سبسکرائبرز
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.command_callback, 10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped, 'robot_pose', self.pose_callback, 10
        )

        # سسٹم پرامپٹ
        self.system_prompt = self._build_system_prompt()

        self.get_logger().info('LLM پلانر نوڈ شروع ہو گیا')

    def _build_system_prompt(self) -> str:
        """روبوٹ پلاننگ کے لیے سسٹم پرامپٹ بنائیں۔"""
        return """آپ ذہین روبوٹ پلانر ہیں۔ قدرتی زبان کی کمانڈز کو قابل عمل روبوٹ ایکشن سیکوینسز میں تبدیل کریں۔

## دستیاب روبوٹ ایکشنز
1. NAVIGATE(x, y, theta) - پوزیشن پر جائیں
2. PICK(object_name) - چیز پکڑیں
3. PLACE(location) - پکڑی ہوئی چیز رکھیں
4. LOOK(direction) - کیمرہ موڑیں
5. WAIT(seconds) - عمل روکیں
6. SAY(text) - یوزر سے بولیں

## جواب کی شکل
JSON سے جواب دیں:
{
  "reasoning": "پلان کی مختصر وضاحت",
  "actions": [
    {"action": "ACTION_NAME", "params": {...}, "step": 1}
  ],
  "estimated_time": seconds,
  "risks": ["ممکنہ مسائل"]
}

## سیفٹی قواعد
- رکاوٹوں میں سے نہ گزریں
- اٹھانے سے پہلے چیز پکڑنے کے قابل ہونا تصدیق کریں
- خطرناک ایکشنز یوزر سے کنفرم کریں
- ورک سپیس حدود میں رہیں"""

    def pose_callback(self, msg: PoseStamped):
        """پوز پیغام سے روبوٹ پوزیشن اپڈیٹ کریں۔"""
        self.robot_state['position'] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }

    def command_callback(self, msg: String):
        """آنے والی کمانڈ پروسیس کریں اور پلان بنائیں۔"""
        command = msg.data
        self.get_logger().info(f'کمانڈ موصول: {command}')

        # اسٹیٹس پبلش کریں
        status_msg = String()
        status_msg.data = "planning"
        self.status_pub.publish(status_msg)

        try:
            # LLM سے پلان بنائیں
            plan = self._generate_plan(command)

            # پلان پبلش کریں
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

            self.get_logger().info(f'{len(plan.get("actions", []))} ایکشنز کے ساتھ پلان بنایا')

            # اسٹیٹس اپڈیٹ کریں
            status_msg.data = "ready"
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'پلاننگ ناکام: {e}')
            status_msg.data = "error"
            self.status_pub.publish(status_msg)

    def _generate_plan(self, command: str) -> Dict:
        """LLM استعمال کرتے ہوئے ٹاسک پلان بنائیں۔"""
        context = {
            "robot_state": self.robot_state,
            "timestamp": self.get_clock().now().to_msg()
        }

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": f"کمانڈ: {command}\n\nسیاق: {json.dumps(context)}"}
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

## ٹاسک ڈیکمپوزیشن (Task Decomposition)

پیچیدہ ٹاسکس کو ذیلی ٹاسکس میں توڑیں:

```python title="task_decomposer.py"
#!/usr/bin/env python3
from typing import List, Dict
from dataclasses import dataclass
from openai import OpenAI

@dataclass
class SubTask:
    """ڈیکمپوزڈ ذیلی ٹاسک۔"""
    id: int
    description: str
    prerequisites: List[int]
    actions: List[Dict]
    verification: str

class TaskDecomposer:
    """پیچیدہ ٹاسکس کو قابل انتظام ذیلی ٹاسکس میں توڑیں۔"""

    def __init__(self, client: OpenAI, model: str = "gpt-4"):
        self.client = client
        self.model = model

    def decompose(self, task: str, context: Dict = None) -> List[SubTask]:
        """
        پیچیدہ ٹاسک کو ذیلی ٹاسکس میں توڑیں۔

        Args:
            task: اعلی سطحی ٹاسک کی تفصیل
            context: ماحول اور روبوٹ کا سیاق

        Returns:
            SubTask آبجیکٹس کی فہرست
        """
        prompt = f"""اس روبوٹ ٹاسک کو ذیلی ٹاسکس میں توڑیں:

ٹاسک: {task}

ہر ذیلی ٹاسک کے لیے فراہم کریں:
1. تفصیل
2. پیش شرائط (کون سے ذیلی ٹاسکس پہلے مکمل ہونے چاہئیں)
3. مخصوص روبوٹ ایکشنز
4. تکمیل کی تصدیق کیسے کریں

JSON کے طور پر آؤٹ پٹ کریں:
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
                {"role": "system", "content": "آپ روبوٹ ٹاسک پلانر ہیں۔"},
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

# مثال: "کافی بنائیں" ٹاسک کو توڑیں
if __name__ == "__main__":
    import os

    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    decomposer = TaskDecomposer(client)

    subtasks = decomposer.decompose("ایک کپ کافی بنائیں")

    print("ٹاسک ڈیکمپوزیشن:")
    for st in subtasks:
        print(f"\n{st.id}. {st.description}")
        print(f"   پیش شرائط: {st.prerequisites}")
        print(f"   ایکشنز: {st.actions}")
        print(f"   تصدیق: {st.verification}")
```

## سیفٹی ویلیڈیشن (Safety Validation)

عمل سے پہلے LLM آؤٹ پٹس کی تصدیق کریں:

```python title="safety_validator.py"
#!/usr/bin/env python3
from typing import List, Dict, Tuple
from dataclasses import dataclass

@dataclass
class ValidationResult:
    """سیفٹی ویلیڈیشن کا نتیجہ۔"""
    is_safe: bool
    issues: List[str]
    modified_plan: Dict = None

class SafetyValidator:
    """سیفٹی کے لیے روبوٹ ایکشن پلانز کی تصدیق کریں۔"""

    def __init__(self):
        # ورک سپیس حدود بیان کریں
        self.workspace = {
            'x_min': -5.0, 'x_max': 5.0,
            'y_min': -5.0, 'y_max': 5.0,
            'z_min': 0.0, 'z_max': 2.0
        }

        # زیادہ سے زیادہ ویلوسٹیز
        self.max_linear_vel = 1.0  # m/s
        self.max_angular_vel = 1.5  # rad/s

        # ممنوعہ زونز (مثلاً انسانوں کے قریب)
        self.forbidden_zones = []

        # تصدیق کی ضرورت والے خطرناک ایکشنز
        self.dangerous_actions = ['DROP', 'THROW', 'PUSH']

    def validate(self, plan: Dict) -> ValidationResult:
        """
        روبوٹ ایکشن پلان کی تصدیق کریں۔

        Args:
            plan: LLM سے ایکشن پلان

        Returns:
            سیفٹی تشخیص کے ساتھ ValidationResult
        """
        issues = []

        actions = plan.get('actions', [])

        for i, action in enumerate(actions):
            action_issues = self._validate_action(action, i)
            issues.extend(action_issues)

        # خطرناک ایکشنز چیک کریں
        dangerous = self._check_dangerous_actions(actions)
        issues.extend(dangerous)

        return ValidationResult(
            is_safe=len(issues) == 0,
            issues=issues
        )

    def _validate_action(self, action: Dict, index: int) -> List[str]:
        """واحد ایکشن کی تصدیق کریں۔"""
        issues = []
        action_type = action.get('action', '')
        params = action.get('params', {})

        # نیویگیشن ٹارگٹس چیک کریں
        if action_type == 'NAVIGATE':
            x, y = params.get('x', 0), params.get('y', 0)

            if not self._in_workspace(x, y):
                issues.append(
                    f"ایکشن {index}: نیویگیشن ٹارگٹ ({x}, {y}) ورک سپیس سے باہر"
                )

            if self._in_forbidden_zone(x, y):
                issues.append(
                    f"ایکشن {index}: نیویگیشن ٹارگٹ ({x}, {y}) ممنوعہ زون میں"
                )

        # ویلوسٹی حدود چیک کریں
        if action_type == 'MOVE':
            vel = params.get('velocity', 0)
            if vel > self.max_linear_vel:
                issues.append(
                    f"ایکشن {index}: ویلوسٹی {vel} حد {self.max_linear_vel} سے زیادہ"
                )

        return issues

    def _in_workspace(self, x: float, y: float) -> bool:
        """چیک کریں کہ پوزیشن ورک سپیس میں ہے۔"""
        return (
            self.workspace['x_min'] <= x <= self.workspace['x_max'] and
            self.workspace['y_min'] <= y <= self.workspace['y_max']
        )

    def _in_forbidden_zone(self, x: float, y: float) -> bool:
        """چیک کریں کہ پوزیشن ممنوعہ زون میں ہے۔"""
        for zone in self.forbidden_zones:
            if (zone['x_min'] <= x <= zone['x_max'] and
                zone['y_min'] <= y <= zone['y_max']):
                return True
        return False

    def _check_dangerous_actions(self, actions: List[Dict]) -> List[str]:
        """خطرناک ایکشنز چیک کریں۔"""
        issues = []

        for i, action in enumerate(actions):
            if action.get('action') in self.dangerous_actions:
                issues.append(
                    f"ایکشن {i}: '{action['action']}' کو یوزر تصدیق کی ضرورت ہے"
                )

        return issues
```

## کنورسیشنل روبوٹ انٹرفیس (Conversational Robot Interface)

ملٹی ٹرن گفتگو فعال کریں:

```python title="conversation_manager.py"
#!/usr/bin/env python3
from typing import List, Dict
from dataclasses import dataclass, field
from openai import OpenAI
import json

@dataclass
class ConversationTurn:
    """گفتگو میں ایک ٹرن۔"""
    role: str
    content: str
    timestamp: float = 0.0

class ConversationManager:
    """روبوٹ کے ساتھ ملٹی ٹرن گفتگو کا انتظام کریں۔"""

    def __init__(self, client: OpenAI, model: str = "gpt-4"):
        self.client = client
        self.model = model
        self.history: List[ConversationTurn] = []
        self.max_history = 20  # آخری 20 ٹرنز رکھیں

        self.system_prompt = """آپ مددگار روبوٹ اسسٹنٹ ہیں۔ آپ یہ کر سکتے ہیں:
1. فزیکل ٹاسکس انجام دینا (چلنا، اٹھانا، رکھنا)
2. ماحول کے بارے میں سوالات کے جواب دینا
3. مبہم کمانڈز واضح کرنا
4. اپنے کام کی وضاحت کرنا

جب آپ کو ایکشنز انجام دینے ہوں، اس طرح جواب دیں:
{
  "response": "یوزر کو قدرتی زبان کا جواب",
  "actions": [روبوٹ ایکشنز کی فہرست],
  "clarification_needed": false
}

جب آپ کو وضاحت چاہیے:
{
  "response": "یوزر کے لیے سوال",
  "actions": [],
  "clarification_needed": true
}"""

    def process_input(self, user_input: str, context: Dict = None) -> Dict:
        """
        یوزر ان پٹ پروسیس کریں اور جواب بنائیں۔

        Args:
            user_input: یوزر کا پیغام
            context: موجودہ روبوٹ/ماحول کا سیاق

        Returns:
            ٹیکسٹ اور اختیاری ایکشنز کے ساتھ جواب Dict
        """
        # یوزر پیغام ہسٹری میں شامل کریں
        self.history.append(ConversationTurn(
            role="user",
            content=user_input
        ))

        # API کے لیے پیغامات بنائیں
        messages = [{"role": "system", "content": self.system_prompt}]

        # سیاق شامل کریں
        if context:
            messages.append({
                "role": "system",
                "content": f"موجودہ سیاق: {json.dumps(context)}"
            })

        # گفتگو کی ہسٹری شامل کریں
        for turn in self.history[-self.max_history:]:
            messages.append({
                "role": turn.role,
                "content": turn.content
            })

        # LLM سے پوچھیں
        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            response_format={"type": "json_object"}
        )

        result = json.loads(response.choices[0].message.content)

        # اسسٹنٹ جواب ہسٹری میں شامل کریں
        self.history.append(ConversationTurn(
            role="assistant",
            content=result.get("response", "")
        ))

        return result

    def reset(self):
        """گفتگو کی ہسٹری صاف کریں۔"""
        self.history = []

# مثال کی گفتگو
if __name__ == "__main__":
    import os

    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    manager = ConversationManager(client)

    context = {
        "objects": ["cup", "plate", "fork"],
        "robot_location": "kitchen"
    }

    # ملٹی ٹرن گفتگو
    inputs = [
        "آپ کیا دیکھ سکتے ہیں؟",
        "کپ اٹھاؤ",
        "اسے کہاں رکھوں؟",
        "میز پر رکھ دو"
    ]

    for user_input in inputs:
        print(f"\nیوزر: {user_input}")
        response = manager.process_input(user_input, context)
        print(f"روبوٹ: {response['response']}")
        if response.get('actions'):
            print(f"ایکشنز: {response['actions']}")
```

## خلاصہ (Summary)

- **LLM انٹیگریشن** قدرتی زبان روبوٹ کنٹرول فعال کرتا ہے
- **ٹاسک ڈیکمپوزیشن** پیچیدہ ٹاسکس کو قابل عمل مراحل میں توڑتا ہے
- **سیفٹی ویلیڈیشن** خطرناک ایکشنز روکتی ہے
- **کنورسیشنل انٹرفیسز** ملٹی ٹرن تعامل کی اجازت دیتے ہیں
- **لوکل LLMs** پرائیویسی اور آف لائن آپریشن فراہم کرتے ہیں

**مبارکباد!** آپ نے Physical AI & Humanoid Robotics نصابی کتاب مکمل کر لی۔ اب آپ کے پاس ROS2، سمیولیشن، NVIDIA Isaac، اور جدید AI ماڈلز استعمال کرتے ہوئے ذہین روبوٹس بنانے کی بنیاد ہے۔
