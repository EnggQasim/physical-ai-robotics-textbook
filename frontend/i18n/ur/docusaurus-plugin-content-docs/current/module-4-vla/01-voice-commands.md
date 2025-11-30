---
sidebar_position: 2
title: "روبوٹس کے لیے وائس کمانڈز"
description: "روبوٹکس کے لیے تقریر کی شناخت اور وائس کنٹرول نافذ کرنا"
---

# روبوٹس کے لیے وائس کمانڈز (Voice Commands for Robots)

وائس کنٹرول انسان-روبوٹ تعامل کے لیے قدرتی انٹرفیس فراہم کرتا ہے۔ اس سیکشن میں روبوٹک سسٹمز کے لیے تقریر کی شناخت اور وائس کمانڈ پروسیسنگ نافذ کرنا شامل ہے۔

## سیکھنے کے مقاصد (Learning Objectives)

- روبوٹ کنٹرول کے لیے سپیچ ٹو ٹیکسٹ نافذ کریں
- قدرتی زبان کی کمانڈز پروسیس اور پارس کریں
- روبوٹس کے ساتھ ملٹی ٹرن گفتگو سنبھالیں
- مضبوط وائس انٹرفیسز بنائیں

## تقریر کی شناخت پائپ لائن (Speech Recognition Pipeline)

```text
┌─────────────────────────────────────────────────────────────┐
│                 وائس کمانڈ پائپ لائن                         │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────────┐   │
│  │ مائیکروفون │──►│   VAD       │──►│    تقریر        │   │
│  │   ان پٹ    │   │ (ڈیٹیکشن)  │   │   ریکگنیشن     │   │
│  └─────────────┘   └─────────────┘   └────────┬────────┘   │
│                                               │             │
│                                               ▼             │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────────┐   │
│  │   روبوٹ    │◄──│   انٹینٹ   │◄──│   NLU           │   │
│  │   ایکشن    │   │  ایگزیکیوٹر │   │   پروسیسنگ     │   │
│  └─────────────┘   └─────────────┘   └─────────────────┘   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## تقریر کی شناخت کے لیے OpenAI Whisper

**Whisper** جدید ترین تقریر کی شناخت کا ماڈل ہے:

### انسٹالیشن

```bash
# Whisper انسٹال کریں
pip install openai-whisper

# یا آپٹیمائزڈ انفرنس کے لیے faster-whisper استعمال کریں
pip install faster-whisper
```

### بنیادی استعمال

```python title="whisper_demo.py"
#!/usr/bin/env python3
import whisper

# ماڈل لوڈ کریں (tiny, base, small, medium, large)
model = whisper.load_model("base")

# آڈیو فائل ٹرانسکرائب کریں
result = model.transcribe("audio.wav")
print(result["text"])

# زبان کی شناخت کے ساتھ
result = model.transcribe("audio.wav", language="en")
print(f"شناخت شدہ زبان: {result['language']}")
print(f"ٹرانسکرپشن: {result['text']}")
```

### ریئل ٹائم ٹرانسکرپشن

```python title="realtime_whisper.py"
#!/usr/bin/env python3
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel

class RealtimeTranscriber:
    """Whisper استعمال کرتے ہوئے ریئل ٹائم تقریر ٹرانسکرپشن۔"""

    def __init__(self, model_size="base"):
        # بہتر کارکردگی کے لیے faster-whisper استعمال کریں
        self.model = WhisperModel(
            model_size,
            device="cuda",  # یا "cpu"
            compute_type="float16"
        )

        self.sample_rate = 16000
        self.chunk_duration = 3.0  # سیکنڈز
        self.buffer = []

    def audio_callback(self, indata, frames, time, status):
        """ہر آڈیو چنک کے لیے کال ہوتا ہے۔"""
        if status:
            print(f"آڈیو سٹیٹس: {status}")
        self.buffer.extend(indata[:, 0])

    def transcribe_chunk(self, audio_data):
        """آڈیو چنک ٹرانسکرائب کریں۔"""
        audio_np = np.array(audio_data, dtype=np.float32)

        segments, info = self.model.transcribe(
            audio_np,
            beam_size=5,
            language="en",
            vad_filter=True
        )

        text = " ".join([seg.text for seg in segments])
        return text.strip()

    def run(self):
        """ریئل ٹائم ٹرانسکرپشن شروع کریں۔"""
        print("سن رہا ہوں... (روکنے کے لیے Ctrl+C دبائیں)")

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=self.audio_callback
        ):
            while True:
                # کافی آڈیو کا انتظار کریں
                if len(self.buffer) >= self.sample_rate * self.chunk_duration:
                    # چنک حاصل کریں اور بفر صاف کریں
                    chunk = self.buffer[:int(self.sample_rate * self.chunk_duration)]
                    self.buffer = self.buffer[int(self.sample_rate * self.chunk_duration):]

                    # ٹرانسکرائب کریں
                    text = self.transcribe_chunk(chunk)
                    if text:
                        print(f"ٹرانسکرائبڈ: {text}")

if __name__ == "__main__":
    transcriber = RealtimeTranscriber()
    transcriber.run()
```

## وائس ایکٹیویٹی ڈیٹیکشن (VAD)

تقریر کب شروع اور ختم ہوتی ہے معلوم کریں:

```python title="vad_detector.py"
#!/usr/bin/env python3
import webrtcvad
import numpy as np

class VoiceActivityDetector:
    """WebRTC VAD استعمال کرتے ہوئے وائس ایکٹیویٹی ڈیٹیکشن۔"""

    def __init__(self, aggressiveness=2):
        """
        VAD شروع کریں۔

        Args:
            aggressiveness: 0-3، زیادہ = زیادہ جارحانہ فلٹرنگ
        """
        self.vad = webrtcvad.Vad(aggressiveness)
        self.sample_rate = 16000
        self.frame_duration = 30  # ms

    def is_speech(self, audio_frame):
        """
        چیک کریں کہ آڈیو فریم میں تقریر ہے۔

        Args:
            audio_frame: خام آڈیو بائٹس (16-bit PCM)

        Returns:
            bool: اگر تقریر ملی تو True
        """
        return self.vad.is_speech(audio_frame, self.sample_rate)

    def process_audio(self, audio_data):
        """
        آڈیو پروسیس کریں اور تقریر سیگمنٹس واپس کریں۔

        Args:
            audio_data: آڈیو سیمپلز کی numpy ارے

        Returns:
            list: تقریر پر مشتمل سیگمنٹس
        """
        # 16-bit PCM میں تبدیل کریں
        audio_bytes = (audio_data * 32767).astype(np.int16).tobytes()

        frame_size = int(self.sample_rate * self.frame_duration / 1000) * 2
        speech_segments = []
        current_segment = []
        in_speech = False

        for i in range(0, len(audio_bytes), frame_size):
            frame = audio_bytes[i:i + frame_size]
            if len(frame) < frame_size:
                break

            is_speech = self.is_speech(frame)

            if is_speech and not in_speech:
                # تقریر شروع ہوئی
                in_speech = True
                current_segment = [frame]
            elif is_speech and in_speech:
                # تقریر جاری ہے
                current_segment.append(frame)
            elif not is_speech and in_speech:
                # تقریر ختم ہوئی
                in_speech = False
                speech_segments.append(b''.join(current_segment))
                current_segment = []

        return speech_segments
```

## ROS2 وائس کمانڈ نوڈ

ROS2 سے انٹیگریشن:

```python title="voice_command_node.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel

class VoiceCommandNode(Node):
    """وائس کمانڈ ریکگنیشن کے لیے ROS2 نوڈ۔"""

    def __init__(self):
        super().__init__('voice_command_node')

        # پیرامیٹرز
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('language', 'en')
        self.declare_parameter('sample_rate', 16000)

        model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value

        # Whisper ماڈل لوڈ کریں
        self.get_logger().info(f'Whisper ماڈل لوڈ ہو رہا ہے: {model_size}')
        self.model = WhisperModel(model_size, device="cuda")

        # پبلشرز
        self.command_pub = self.create_publisher(String, 'voice_command', 10)
        self.text_pub = self.create_publisher(String, 'transcription', 10)

        # سننا شروع/بند کرنے کی سروس
        self.listen_srv = self.create_service(
            Trigger, 'start_listening', self.start_listening_callback
        )

        # آڈیو اسٹیٹ
        self.is_listening = False
        self.audio_buffer = []
        self.silence_frames = 0
        self.max_silence = 30  # ریکارڈنگ ختم کرنے کے لیے خاموشی کے فریمز

        self.get_logger().info('وائس کمانڈ نوڈ شروع ہو گیا')

    def start_listening_callback(self, request, response):
        """وائس کمانڈز سننا شروع کریں۔"""
        if self.is_listening:
            response.success = False
            response.message = "پہلے سے سن رہا ہے"
            return response

        self.is_listening = True
        self.audio_buffer = []
        self.listen_for_command()

        response.success = True
        response.message = "سننا شروع ہو گیا"
        return response

    def audio_callback(self, indata, frames, time, status):
        """آنے والا آڈیو پروسیس کریں۔"""
        if not self.is_listening:
            return

        # سادہ انرجی بیسڈ VAD
        energy = np.abs(indata).mean()

        if energy > 0.01:  # تقریر ملی
            self.audio_buffer.extend(indata[:, 0])
            self.silence_frames = 0
        elif len(self.audio_buffer) > 0:
            self.silence_frames += 1
            self.audio_buffer.extend(indata[:, 0])

            # تقریر کا اختتام
            if self.silence_frames > self.max_silence:
                self.process_command()

    def listen_for_command(self):
        """کمانڈ کیپچر کے لیے آڈیو سٹریم شروع کریں۔"""
        self.get_logger().info('وائس کمانڈ سن رہا ہوں...')

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=self.audio_callback
        ):
            # کمانڈ پروسیس ہونے تک سٹریم کھلی رکھیں
            while self.is_listening:
                sd.sleep(100)

    def process_command(self):
        """کیپچر کردہ آڈیو پروسیس کریں اور کمانڈ نکالیں۔"""
        self.is_listening = False

        if len(self.audio_buffer) < self.sample_rate:  # 1 سیکنڈ سے کم
            self.get_logger().info('آڈیو بہت مختصر، نظرانداز کر رہا ہوں')
            return

        # numpy ارے میں تبدیل کریں
        audio_np = np.array(self.audio_buffer, dtype=np.float32)

        # ٹرانسکرائب کریں
        self.get_logger().info('آڈیو ٹرانسکرائب ہو رہا ہے...')
        segments, _ = self.model.transcribe(
            audio_np,
            language=self.language,
            beam_size=5
        )

        text = " ".join([seg.text for seg in segments]).strip()

        if text:
            self.get_logger().info(f'ٹرانسکرائبڈ: {text}')

            # ٹرانسکرپشن پبلش کریں
            text_msg = String()
            text_msg.data = text
            self.text_pub.publish(text_msg)

            # کمانڈ کے طور پر پبلش کریں
            cmd_msg = String()
            cmd_msg.data = text
            self.command_pub.publish(cmd_msg)

def main():
    rclpy.init()
    node = VoiceCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## انٹینٹ پارسنگ (Intent Parsing)

قدرتی زبان سے ڈھانچے دار انٹینٹس نکالیں:

```python title="intent_parser.py"
#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Optional, Dict, Any
import re

@dataclass
class RobotIntent:
    """ڈھانچے دار روبوٹ کمانڈ انٹینٹ۔"""
    action: str
    target: Optional[str] = None
    location: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None

class IntentParser:
    """قدرتی زبان کمانڈز کو روبوٹ انٹینٹس میں پارس کریں۔"""

    def __init__(self):
        # ایکشن پیٹرنز بیان کریں
        self.action_patterns = {
            'move': r'\b(move|go|navigate|drive)\b',
            'pick': r'\b(pick|grab|grasp|take|get)\b',
            'place': r'\b(place|put|set|drop)\b',
            'stop': r'\b(stop|halt|freeze|pause)\b',
            'look': r'\b(look|see|find|locate|search)\b',
        }

        # ٹارگٹ پیٹرنز بیان کریں
        self.target_patterns = {
            'object': r'the\s+(\w+(?:\s+\w+)?)',
            'color_object': r'the\s+(red|blue|green|yellow|black|white)\s+(\w+)',
        }

        # لوکیشن پیٹرنز بیان کریں
        self.location_patterns = {
            'to': r'to\s+(?:the\s+)?(\w+)',
            'from': r'from\s+(?:the\s+)?(\w+)',
            'near': r'near\s+(?:the\s+)?(\w+)',
            'on': r'on\s+(?:the\s+)?(\w+)',
        }

    def parse(self, text: str) -> Optional[RobotIntent]:
        """
        قدرتی زبان کمانڈ کو ڈھانچے دار انٹینٹ میں پارس کریں۔

        Args:
            text: قدرتی زبان کمانڈ

        Returns:
            RobotIntent یا None اگر پارسنگ ناکام ہو
        """
        text = text.lower().strip()

        # ایکشن تلاش کریں
        action = None
        for action_name, pattern in self.action_patterns.items():
            if re.search(pattern, text):
                action = action_name
                break

        if action is None:
            return None

        # ٹارگٹ تلاش کریں
        target = None
        for pattern_name, pattern in self.target_patterns.items():
            match = re.search(pattern, text)
            if match:
                target = match.group(1) if match.lastindex == 1 else f"{match.group(1)} {match.group(2)}"
                break

        # لوکیشن تلاش کریں
        location = None
        for loc_type, pattern in self.location_patterns.items():
            match = re.search(pattern, text)
            if match:
                location = match.group(1)
                break

        return RobotIntent(
            action=action,
            target=target,
            location=location,
            parameters={}
        )

# مثال کا استعمال
if __name__ == "__main__":
    parser = IntentParser()

    commands = [
        "Move to the kitchen",
        "Pick up the red cup",
        "Place the box on the table",
        "Stop",
        "Find the blue ball",
    ]

    for cmd in commands:
        intent = parser.parse(cmd)
        print(f"کمانڈ: '{cmd}'")
        print(f"  انٹینٹ: {intent}")
        print()
```

## کمانڈ ایگزیکیوٹر (Command Executor)

پارس کردہ انٹینٹس کو انجام دیں:

```python title="command_executor.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose

class CommandExecutor(Node):
    """پارس کردہ انٹینٹس سے روبوٹ کمانڈز انجام دیں۔"""

    def __init__(self):
        super().__init__('command_executor')

        # ایکشن کلائنٹس
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # پبلشرز
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # وائس کمانڈز سبسکرائب کریں
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.command_callback, 10
        )

        # انٹینٹ پارسر
        from intent_parser import IntentParser
        self.parser = IntentParser()

        # معروف لوکیشنز
        self.locations = {
            'kitchen': {'x': 5.0, 'y': 2.0},
            'living room': {'x': 0.0, 'y': 0.0},
            'bedroom': {'x': -3.0, 'y': 4.0},
            'table': {'x': 2.0, 'y': 1.0},
        }

        self.get_logger().info('کمانڈ ایگزیکیوٹر شروع ہو گیا')

    def command_callback(self, msg):
        """آنے والی وائس کمانڈ پروسیس کریں۔"""
        text = msg.data
        self.get_logger().info(f'کمانڈ موصول: {text}')

        # انٹینٹ پارس کریں
        intent = self.parser.parse(text)

        if intent is None:
            self.get_logger().warn(f'کمانڈ پارس نہیں ہو سکی: {text}')
            return

        self.get_logger().info(f'پارس کردہ انٹینٹ: {intent}')

        # ایکشن کی بنیاد پر عمل کریں
        if intent.action == 'move':
            self.execute_move(intent)
        elif intent.action == 'stop':
            self.execute_stop()
        elif intent.action == 'pick':
            self.execute_pick(intent)
        elif intent.action == 'place':
            self.execute_place(intent)
        else:
            self.get_logger().warn(f'نامعلوم ایکشن: {intent.action}')

    def execute_move(self, intent):
        """نیویگیشن کمانڈ انجام دیں۔"""
        location = intent.location

        if location not in self.locations:
            self.get_logger().warn(f'نامعلوم لوکیشن: {location}')
            return

        coords = self.locations[location]
        self.get_logger().info(f'{location} کی طرف جا رہا ہے: {coords}')

        # نیویگیشن گول بنائیں
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = coords['x']
        goal_msg.pose.pose.position.y = coords['y']
        goal_msg.pose.pose.orientation.w = 1.0

        # گول بھیجیں
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)

    def execute_stop(self):
        """روبوٹ روکیں۔"""
        self.get_logger().info('روبوٹ رک رہا ہے')

        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def execute_pick(self, intent):
        """پک کمانڈ انجام دیں۔"""
        target = intent.target
        self.get_logger().info(f'اٹھا رہا ہے: {target}')
        # مینیپولیشن لاجک یہاں نافذ کریں

    def execute_place(self, intent):
        """پلیس کمانڈ انجام دیں۔"""
        target = intent.target
        location = intent.location
        self.get_logger().info(f'{target} کو {location} پر رکھ رہا ہے')
        # مینیپولیشن لاجک یہاں نافذ کریں

def main():
    rclpy.init()
    node = CommandExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ویک ورڈ ڈیٹیکشن (Wake Word Detection)

ہمیشہ آن ویک ورڈ ڈیٹیکشن نافذ کریں:

```python title="wake_word.py"
#!/usr/bin/env python3
import pvporcupine
import sounddevice as sd
import numpy as np

class WakeWordDetector:
    """Porcupine استعمال کرتے ہوئے ویک ورڈز کا پتہ لگائیں۔"""

    def __init__(self, access_key, keywords=["hey robot"]):
        """
        ویک ورڈ ڈیٹیکٹر شروع کریں۔

        Args:
            access_key: Picovoice ایکسیس کی
            keywords: پتہ لگانے والے ویک ورڈز کی فہرست
        """
        self.porcupine = pvporcupine.create(
            access_key=access_key,
            keywords=keywords
        )

        self.sample_rate = self.porcupine.sample_rate
        self.frame_length = self.porcupine.frame_length

        self.callback = None

    def set_callback(self, callback):
        """ویک ورڈ ملنے پر کال کرنے کا فنکشن سیٹ کریں۔"""
        self.callback = callback

    def audio_callback(self, indata, frames, time, status):
        """ویک ورڈ کے لیے آڈیو پروسیس کریں۔"""
        # int16 میں تبدیل کریں
        audio_frame = (indata[:, 0] * 32767).astype(np.int16)

        # Porcupine سے پروسیس کریں
        keyword_index = self.porcupine.process(audio_frame)

        if keyword_index >= 0:
            print("ویک ورڈ ملا!")
            if self.callback:
                self.callback()

    def run(self):
        """ویک ورڈ ڈیٹیکشن شروع کریں۔"""
        print("ویک ورڈ سن رہا ہوں...")

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            blocksize=self.frame_length,
            callback=self.audio_callback
        ):
            while True:
                sd.sleep(100)

    def cleanup(self):
        """وسائل جاری کریں۔"""
        self.porcupine.delete()
```

## خلاصہ (Summary)

- **Whisper** درست سپیچ ٹو ٹیکسٹ ٹرانسکرپشن فراہم کرتا ہے
- **VAD** مؤثر طریقے سے تقریر کی حدود کا پتہ لگاتا ہے
- **انٹینٹ پارسنگ** ٹیکسٹ سے ڈھانچے دار کمانڈز نکالتی ہے
- **ROS2 انٹیگریشن** وائس کے ذریعے روبوٹ کنٹرول فعال کرتا ہے
- **ویک ورڈز** ہینڈز فری ایکٹیویشن فعال کرتے ہیں

**اگلا**: [LLM انٹیگریشن](./llm-integration) - بڑے لینگویج ماڈلز کو روبوٹس سے جوڑیں۔
