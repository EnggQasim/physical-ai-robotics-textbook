---
sidebar_position: 3
title: "سروسز اور ایکشنز"
description: "ROS2 سروسز کے ساتھ request-response پیٹرنز اور ایکشنز کے ساتھ طویل عرصے کے ٹاسکس"
---

# ROS2 سروسز اور ایکشنز

جبکہ ٹاپکس مسلسل ڈیٹا سٹریمز کے لیے بہترین ہیں، بہت سے روبوٹکس ٹاسکس کو **request-response** کمیونیکیشن یا **فیڈبیک کے ساتھ طویل عرصے کے آپریشنز** کی ضرورت ہوتی ہے۔ یہیں **سروسز** (services) اور **ایکشنز** (actions) کام آتے ہیں۔

## سیکھنے کے مقاصد

- سمجھنا کہ سروسز، ٹاپکس، اور ایکشنز میں سے کب کیا استعمال کرنا ہے
- ROS2 سروس سرورز اور کلائنٹس بنانا
- طویل عرصے کے ٹاسکس کے لیے ایکشن سرورز کا نفاذ
- ایکشنز میں فیڈبیک اور منسوخی کو سنبھالنا

## کب کیا استعمال کریں؟

| پیٹرن | استعمال کا معاملہ | مثال |
|-------|------------------|------|
| **ٹاپکس** | مسلسل ڈیٹا سٹریمز | سینسر ڈیٹا، رفتار کے کمانڈز |
| **سروسز** | فوری request-response | روبوٹ کی حالت حاصل کرنا، پیرامیٹرز سیٹ کرنا |
| **ایکشنز** | فیڈبیک کے ساتھ طویل ٹاسکس | نیویگیشن، بازو کی حرکت |

## ROS2 سروسز

سروسز **ہم آہنگ request-response** کمیونیکیشن فراہم کرتی ہیں:

![ROS2 Service-Client Communication](/img/generated/ros2/service-diagram.svg)
*تصویر: ROS2 سروس-کلائنٹ پیٹرن - کلائنٹ سروس سرور کو درخواست بھیجتا ہے اور جواب کا انتظار کرتا ہے۔*

### سروس کی خصوصیات

- **ہم آہنگ** (Synchronous): کلائنٹ جواب کا انتظار کرتا ہے
- **ایک سے ایک** (One-to-one): ایک درخواست، ایک جواب
- **ٹائپ شدہ** (Typed): `.srv` فائلوں سے وضاحت شدہ
- **مختصر مدت** (Short-lived): فوری آپریشنز کے لیے

### سروس کی تعریف

سروسز `.srv` فائلوں میں بیان کی جاتی ہیں، درخواست اور جواب `---` سے الگ:

```text title="AddTwoInts.srv"
# درخواست
int64 a
int64 b
---
# جواب
int64 sum
```

### سروس سرور بنانا

```python title="add_two_ints_server.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    """سادہ سروس سرور جو دو نمبر جوڑتا ہے۔"""

    def __init__(self):
        super().__init__('add_two_ints_server')

        # سروس بنائیں
        self.srv = self.create_service(
            AddTwoInts,                    # سروس کی قسم
            'add_two_ints',                # سروس کا نام
            self.add_two_ints_callback     # Callback فنکشن
        )

        self.get_logger().info('سروس سرور تیار ہے')

    def add_two_ints_callback(self, request, response):
        """سروس درخواست کو سنبھالیں۔"""
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
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

## ROS2 ایکشنز

ایکشنز طویل عرصے کے ٹاسکس کو فیڈبیک کے ساتھ سنبھالتے ہیں:

### ایکشن کی خصوصیات

- **غیر ہم آہنگ** (Asynchronous): کلائنٹ کام جاری رکھ سکتا ہے
- **فیڈبیک** (Feedback): عمل کے دوران پیش رفت کی اپڈیٹس
- **قابل منسوخ** (Cancellable): ٹاسک کو درمیان میں روکا جا سکتا ہے
- **نتیجہ** (Result): مکمل ہونے پر حتمی آؤٹ پٹ

### ایکشن سرور مثال

```python title="fibonacci_action_server.py"
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    """Fibonacci سیکوئنس حساب کرنے کے لیے ایکشن سرور۔"""

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('Fibonacci ایکشن سرور تیار ہے')

    def execute_callback(self, goal_handle):
        """ایکشن goal کو execute کریں۔"""
        self.get_logger().info('Goal execute ہو رہا ہے...')

        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i-1])

            # فیڈبیک بھیجیں
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        return result
```

## خلاصہ

- **سروسز** فوری request-response کمیونیکیشن کے لیے ہیں
- **ایکشنز** فیڈبیک کے ساتھ طویل ٹاسکس کے لیے ہیں
- ہر پیٹرن کا اپنا استعمال کا معاملہ ہے

**اگلا**: [Python انٹیگریشن](./python-rclpy) - rclpy کے ساتھ ROS2 ایپلیکیشنز بنانا۔
