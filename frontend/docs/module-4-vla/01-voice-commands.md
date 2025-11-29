---
sidebar_position: 2
title: "Voice Commands for Robots"
description: "Implementing speech recognition and voice control for robotics"
---

# Voice Commands for Robots

Voice control provides a natural interface for human-robot interaction. This section covers implementing speech recognition and voice command processing for robotic systems.

## Learning Objectives

- Implement speech-to-text for robot control
- Process and parse natural language commands
- Handle multi-turn conversations with robots
- Build robust voice interfaces

## Speech Recognition Pipeline

```text
┌─────────────────────────────────────────────────────────────┐
│                 Voice Command Pipeline                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────────┐   │
│  │ Microphone  │──►│   VAD       │──►│    Speech       │   │
│  │   Input     │   │ (Detection) │   │   Recognition   │   │
│  └─────────────┘   └─────────────┘   └────────┬────────┘   │
│                                               │             │
│                                               ▼             │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────────┐   │
│  │   Robot     │◄──│   Intent    │◄──│   NLU           │   │
│  │   Action    │   │   Executor  │   │   Processing    │   │
│  └─────────────┘   └─────────────┘   └─────────────────┘   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## OpenAI Whisper for Speech Recognition

**Whisper** is a state-of-the-art speech recognition model:

### Installation

```bash
# Install Whisper
pip install openai-whisper

# Or use faster-whisper for optimized inference
pip install faster-whisper
```

### Basic Usage

```python title="whisper_demo.py"
#!/usr/bin/env python3
import whisper

# Load model (tiny, base, small, medium, large)
model = whisper.load_model("base")

# Transcribe audio file
result = model.transcribe("audio.wav")
print(result["text"])

# With language detection
result = model.transcribe("audio.wav", language="en")
print(f"Detected language: {result['language']}")
print(f"Transcription: {result['text']}")
```

### Real-time Transcription

```python title="realtime_whisper.py"
#!/usr/bin/env python3
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel

class RealtimeTranscriber:
    """Real-time speech transcription using Whisper."""

    def __init__(self, model_size="base"):
        # Use faster-whisper for better performance
        self.model = WhisperModel(
            model_size,
            device="cuda",  # or "cpu"
            compute_type="float16"
        )

        self.sample_rate = 16000
        self.chunk_duration = 3.0  # seconds
        self.buffer = []

    def audio_callback(self, indata, frames, time, status):
        """Called for each audio chunk."""
        if status:
            print(f"Audio status: {status}")
        self.buffer.extend(indata[:, 0])

    def transcribe_chunk(self, audio_data):
        """Transcribe audio chunk."""
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
        """Start real-time transcription."""
        print("Listening... (Press Ctrl+C to stop)")

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=self.audio_callback
        ):
            while True:
                # Wait for enough audio
                if len(self.buffer) >= self.sample_rate * self.chunk_duration:
                    # Get chunk and clear buffer
                    chunk = self.buffer[:int(self.sample_rate * self.chunk_duration)]
                    self.buffer = self.buffer[int(self.sample_rate * self.chunk_duration):]

                    # Transcribe
                    text = self.transcribe_chunk(chunk)
                    if text:
                        print(f"Transcribed: {text}")

if __name__ == "__main__":
    transcriber = RealtimeTranscriber()
    transcriber.run()
```

## Voice Activity Detection (VAD)

Detect when speech starts and ends:

```python title="vad_detector.py"
#!/usr/bin/env python3
import webrtcvad
import numpy as np

class VoiceActivityDetector:
    """Voice Activity Detection using WebRTC VAD."""

    def __init__(self, aggressiveness=2):
        """
        Initialize VAD.

        Args:
            aggressiveness: 0-3, higher = more aggressive filtering
        """
        self.vad = webrtcvad.Vad(aggressiveness)
        self.sample_rate = 16000
        self.frame_duration = 30  # ms

    def is_speech(self, audio_frame):
        """
        Check if audio frame contains speech.

        Args:
            audio_frame: Raw audio bytes (16-bit PCM)

        Returns:
            bool: True if speech detected
        """
        return self.vad.is_speech(audio_frame, self.sample_rate)

    def process_audio(self, audio_data):
        """
        Process audio and return speech segments.

        Args:
            audio_data: numpy array of audio samples

        Returns:
            list: Segments containing speech
        """
        # Convert to 16-bit PCM
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
                # Speech started
                in_speech = True
                current_segment = [frame]
            elif is_speech and in_speech:
                # Speech continues
                current_segment.append(frame)
            elif not is_speech and in_speech:
                # Speech ended
                in_speech = False
                speech_segments.append(b''.join(current_segment))
                current_segment = []

        return speech_segments
```

## ROS2 Voice Command Node

Integration with ROS2:

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
    """ROS2 node for voice command recognition."""

    def __init__(self):
        super().__init__('voice_command_node')

        # Parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('language', 'en')
        self.declare_parameter('sample_rate', 16000)

        model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        self.model = WhisperModel(model_size, device="cuda")

        # Publishers
        self.command_pub = self.create_publisher(String, 'voice_command', 10)
        self.text_pub = self.create_publisher(String, 'transcription', 10)

        # Service to start/stop listening
        self.listen_srv = self.create_service(
            Trigger, 'start_listening', self.start_listening_callback
        )

        # Audio state
        self.is_listening = False
        self.audio_buffer = []
        self.silence_frames = 0
        self.max_silence = 30  # frames of silence to end recording

        self.get_logger().info('Voice command node initialized')

    def start_listening_callback(self, request, response):
        """Start listening for voice commands."""
        if self.is_listening:
            response.success = False
            response.message = "Already listening"
            return response

        self.is_listening = True
        self.audio_buffer = []
        self.listen_for_command()

        response.success = True
        response.message = "Listening started"
        return response

    def audio_callback(self, indata, frames, time, status):
        """Process incoming audio."""
        if not self.is_listening:
            return

        # Simple energy-based VAD
        energy = np.abs(indata).mean()

        if energy > 0.01:  # Speech detected
            self.audio_buffer.extend(indata[:, 0])
            self.silence_frames = 0
        elif len(self.audio_buffer) > 0:
            self.silence_frames += 1
            self.audio_buffer.extend(indata[:, 0])

            # End of speech
            if self.silence_frames > self.max_silence:
                self.process_command()

    def listen_for_command(self):
        """Start audio stream for command capture."""
        self.get_logger().info('Listening for voice command...')

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=self.audio_callback
        ):
            # Keep stream open until command processed
            while self.is_listening:
                sd.sleep(100)

    def process_command(self):
        """Process captured audio and extract command."""
        self.is_listening = False

        if len(self.audio_buffer) < self.sample_rate:  # Less than 1 second
            self.get_logger().info('Audio too short, ignoring')
            return

        # Convert to numpy array
        audio_np = np.array(self.audio_buffer, dtype=np.float32)

        # Transcribe
        self.get_logger().info('Transcribing audio...')
        segments, _ = self.model.transcribe(
            audio_np,
            language=self.language,
            beam_size=5
        )

        text = " ".join([seg.text for seg in segments]).strip()

        if text:
            self.get_logger().info(f'Transcribed: {text}')

            # Publish transcription
            text_msg = String()
            text_msg.data = text
            self.text_pub.publish(text_msg)

            # Publish as command
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

## Intent Parsing

Extract structured intents from natural language:

```python title="intent_parser.py"
#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Optional, Dict, Any
import re

@dataclass
class RobotIntent:
    """Structured robot command intent."""
    action: str
    target: Optional[str] = None
    location: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None

class IntentParser:
    """Parse natural language commands into robot intents."""

    def __init__(self):
        # Define action patterns
        self.action_patterns = {
            'move': r'\b(move|go|navigate|drive)\b',
            'pick': r'\b(pick|grab|grasp|take|get)\b',
            'place': r'\b(place|put|set|drop)\b',
            'stop': r'\b(stop|halt|freeze|pause)\b',
            'look': r'\b(look|see|find|locate|search)\b',
        }

        # Define target patterns
        self.target_patterns = {
            'object': r'the\s+(\w+(?:\s+\w+)?)',
            'color_object': r'the\s+(red|blue|green|yellow|black|white)\s+(\w+)',
        }

        # Define location patterns
        self.location_patterns = {
            'to': r'to\s+(?:the\s+)?(\w+)',
            'from': r'from\s+(?:the\s+)?(\w+)',
            'near': r'near\s+(?:the\s+)?(\w+)',
            'on': r'on\s+(?:the\s+)?(\w+)',
        }

    def parse(self, text: str) -> Optional[RobotIntent]:
        """
        Parse natural language command into structured intent.

        Args:
            text: Natural language command

        Returns:
            RobotIntent or None if parsing fails
        """
        text = text.lower().strip()

        # Find action
        action = None
        for action_name, pattern in self.action_patterns.items():
            if re.search(pattern, text):
                action = action_name
                break

        if action is None:
            return None

        # Find target
        target = None
        for pattern_name, pattern in self.target_patterns.items():
            match = re.search(pattern, text)
            if match:
                target = match.group(1) if match.lastindex == 1 else f"{match.group(1)} {match.group(2)}"
                break

        # Find location
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

# Example usage
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
        print(f"Command: '{cmd}'")
        print(f"  Intent: {intent}")
        print()
```

## Command Executor

Execute parsed intents:

```python title="command_executor.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose

class CommandExecutor(Node):
    """Execute robot commands from parsed intents."""

    def __init__(self):
        super().__init__('command_executor')

        # Action clients
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.command_callback, 10
        )

        # Intent parser
        from intent_parser import IntentParser
        self.parser = IntentParser()

        # Known locations
        self.locations = {
            'kitchen': {'x': 5.0, 'y': 2.0},
            'living room': {'x': 0.0, 'y': 0.0},
            'bedroom': {'x': -3.0, 'y': 4.0},
            'table': {'x': 2.0, 'y': 1.0},
        }

        self.get_logger().info('Command executor initialized')

    def command_callback(self, msg):
        """Process incoming voice command."""
        text = msg.data
        self.get_logger().info(f'Received command: {text}')

        # Parse intent
        intent = self.parser.parse(text)

        if intent is None:
            self.get_logger().warn(f'Could not parse command: {text}')
            return

        self.get_logger().info(f'Parsed intent: {intent}')

        # Execute based on action
        if intent.action == 'move':
            self.execute_move(intent)
        elif intent.action == 'stop':
            self.execute_stop()
        elif intent.action == 'pick':
            self.execute_pick(intent)
        elif intent.action == 'place':
            self.execute_place(intent)
        else:
            self.get_logger().warn(f'Unknown action: {intent.action}')

    def execute_move(self, intent):
        """Execute navigation command."""
        location = intent.location

        if location not in self.locations:
            self.get_logger().warn(f'Unknown location: {location}')
            return

        coords = self.locations[location]
        self.get_logger().info(f'Navigating to {location}: {coords}')

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = coords['x']
        goal_msg.pose.pose.position.y = coords['y']
        goal_msg.pose.pose.orientation.w = 1.0

        # Send goal
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)

    def execute_stop(self):
        """Stop the robot."""
        self.get_logger().info('Stopping robot')

        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def execute_pick(self, intent):
        """Execute pick command."""
        target = intent.target
        self.get_logger().info(f'Picking up: {target}')
        # Implement manipulation logic here

    def execute_place(self, intent):
        """Execute place command."""
        target = intent.target
        location = intent.location
        self.get_logger().info(f'Placing {target} on {location}')
        # Implement manipulation logic here

def main():
    rclpy.init()
    node = CommandExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Wake Word Detection

Implement always-on wake word detection:

```python title="wake_word.py"
#!/usr/bin/env python3
import pvporcupine
import sounddevice as sd
import numpy as np

class WakeWordDetector:
    """Detect wake words using Porcupine."""

    def __init__(self, access_key, keywords=["hey robot"]):
        """
        Initialize wake word detector.

        Args:
            access_key: Picovoice access key
            keywords: List of wake words to detect
        """
        self.porcupine = pvporcupine.create(
            access_key=access_key,
            keywords=keywords
        )

        self.sample_rate = self.porcupine.sample_rate
        self.frame_length = self.porcupine.frame_length

        self.callback = None

    def set_callback(self, callback):
        """Set function to call when wake word detected."""
        self.callback = callback

    def audio_callback(self, indata, frames, time, status):
        """Process audio for wake word."""
        # Convert to int16
        audio_frame = (indata[:, 0] * 32767).astype(np.int16)

        # Process with Porcupine
        keyword_index = self.porcupine.process(audio_frame)

        if keyword_index >= 0:
            print("Wake word detected!")
            if self.callback:
                self.callback()

    def run(self):
        """Start wake word detection."""
        print("Listening for wake word...")

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            blocksize=self.frame_length,
            callback=self.audio_callback
        ):
            while True:
                sd.sleep(100)

    def cleanup(self):
        """Release resources."""
        self.porcupine.delete()
```

## Summary

- **Whisper** provides accurate speech-to-text transcription
- **VAD** detects speech boundaries efficiently
- **Intent parsing** extracts structured commands from text
- **ROS2 integration** enables robot control via voice
- **Wake words** enable hands-free activation

**Next**: [LLM Integration](./llm-integration) - Connect large language models to robots.
