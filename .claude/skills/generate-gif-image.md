---
description: Generate animated GIF images using Google Gemini API to teach difficult concepts with simple visual animations.
---

# Generate GIF Image Skill

This skill generates animated GIF images using Google Gemini 2.5 Flash (Nano Banana) API to visualize complex concepts through simple animations for the Physical AI textbook.

## Purpose

Many robotics and AI concepts are easier to understand when shown as animations rather than static images:
- **Message flow** in ROS2 pub/sub systems
- **Robot motion** sequences and trajectories
- **Data pipelines** processing steps
- **State transitions** in finite state machines
- **Learning loops** in AI training

## Capabilities

- Generate animated GIFs from text prompts (text-to-animation)
- Convert static diagrams to animated sequences (image-to-animation)
- Create looping animations for continuous processes
- Visualize step-by-step workflows
- Animate data flow and message passing

## Prerequisites

```bash
# Install required packages
pip install google-generativeai pillow imageio

# Or use the dedicated gemini-gif tool
pip install gemini-gif
```

**Environment Variable Required**:
```bash
export GEMINI_API_KEY="your-api-key-here"
```

## Usage

### Method 1: Using gemini-gif CLI Tool

```bash
# Basic usage
gemini-gif "ROS2 publisher sending messages to subscriber through topic"

# With custom output
gemini-gif "robot arm picking up object" --output robot_motion.gif

# With style
gemini-gif "neural network forward propagation" --style "technical diagram, minimalist"
```

### Method 2: Using Python Script

When the user requests an animated visualization:

1. **Parse the request** to extract:
   - `concept`: What to animate (required)
   - `style`: Animation style (loop, sequence, flow)
   - `duration`: Approximate length (short: 2s, medium: 4s, long: 6s)
   - `module`: Target module for file organization

2. **Generate the GIF** using the Python script:

```bash
python .specify/scripts/python/gemini_gif.py \
  --prompt "<concept description>" \
  --output "frontend/static/img/animations/<filename>.gif" \
  --style "<style>" \
  --duration "<duration>" \
  --json
```

3. **Integrate into documentation**:
```markdown
![<Alt text>](/img/animations/<filename>.gif)
*Animation: <Caption describing what the animation shows>*
```

## Animation Styles

| Style | Use Case | Example |
|-------|----------|---------|
| `loop` | Continuous processes | ROS2 message publishing loop |
| `sequence` | Step-by-step procedures | Robot boot sequence |
| `flow` | Data/message movement | Topic message flow |
| `transition` | State changes | FSM state transitions |
| `cycle` | Repeating patterns | Control loop iterations |

## Concept Categories for Animation

### ROS2 Concepts (High Value for Animation)
| Concept | Animation Description |
|---------|----------------------|
| Pub/Sub Communication | Publisher node pulsing → message traveling → subscriber receiving |
| Service Call | Client sends request → server processes → response returns |
| Action Execution | Goal sent → feedback streaming → result returned |
| Topic Message Flow | Multiple publishers → topic queue → multiple subscribers |
| Node Lifecycle | Unconfigured → Inactive → Active → Finalized |

### Simulation Concepts
| Concept | Animation Description |
|---------|----------------------|
| Physics Step | Forces applied → collision detection → position update |
| Sensor Scan | LiDAR beam sweeping environment |
| Robot Motion | Joint angles changing over time |
| World Loading | URDF parsing → mesh loading → physics binding |

### NVIDIA Isaac Concepts
| Concept | Animation Description |
|---------|----------------------|
| GPU Parallel Processing | Data splitting → parallel compute → result merging |
| Synthetic Data Generation | Scene randomization → render → annotation |
| Sim-to-Real Transfer | Simulation → domain randomization → real deployment |

### VLA Concepts
| Concept | Animation Description |
|---------|----------------------|
| Vision Processing | Camera input → feature extraction → object detection |
| Language Understanding | Speech → tokenization → embedding → intent |
| Action Generation | Intent → motion planning → joint commands |
| VLA Pipeline | See → Think → Act cycle |

## Output Paths

Save generated GIFs to:
- ROS2 animations: `frontend/static/img/animations/ros2/`
- Simulation animations: `frontend/static/img/animations/simulation/`
- Isaac animations: `frontend/static/img/animations/isaac/`
- VLA animations: `frontend/static/img/animations/vla/`
- General: `frontend/static/img/animations/`

## Theme Requirements

All generated animations should follow:
- **Color Palette**: NVIDIA green `#76b900`, dark `#1a1a1a`, white `#ffffff`
- **Style**: Clean, minimalist, technical illustration
- **Frame Rate**: 10-15 fps for smooth motion
- **Loop**: Seamless looping preferred
- **Size**: Max 2MB per GIF (optimize for web)
- **Dimensions**: 800x600 or 600x400 for inline content

## Example Prompts

### ROS2 Animation
```
Generate a looping GIF showing ROS2 publisher-subscriber communication:
- Left side: Publisher node (green circle) pulsing when sending
- Center: Topic (rectangle) receiving and queuing messages
- Right side: Subscriber node (green circle) receiving messages
- Messages shown as small packets moving left to right
- Use NVIDIA green (#76b900) theme, dark background
```

### Robot Motion Animation
```
Generate a sequence GIF showing a robotic arm pick-and-place:
- Frame 1: Arm in home position
- Frame 2: Arm moving to object
- Frame 3: Gripper closing on object
- Frame 4: Arm lifting object
- Frame 5: Arm moving to target
- Frame 6: Gripper releasing
- Frame 7: Return to home
- Technical diagram style, side view
```

### VLA Pipeline Animation
```
Generate a cycle GIF showing Vision-Language-Action loop:
- Vision: Camera icon capturing image, processing waves
- Language: Text bubble forming, processing into intent
- Action: Robot arm receiving command, executing motion
- Arrows showing cyclic flow between all three
- Loop seamlessly for continuous demonstration
```

## Error Handling

If GIF generation fails:
1. Check `GEMINI_API_KEY` environment variable is set
2. Verify API quota limits (GIF generation uses more quota than images)
3. Try reducing complexity of the prompt
4. Try shorter duration (`--duration short`)
5. Fall back to static SVG diagram with step numbers
6. Fall back to sequence of static images

## Integration with image-generation Skill

This skill complements the `image-generation` skill:
- Use **image-generation** for: Static diagrams, icons, architecture overviews
- Use **generate-gif-image** for: Processes, flows, state changes, motion

## Constitution Alignment

This skill extends **Principle VIII: Visual Learning with AI**:
- Animated visuals for dynamic concepts
- Alt text required for accessibility (describe the animation sequence)
- Stored in `/static/img/animations/` directory
- Follows NVIDIA green theme (#76b900)
- Optimized for web (max 2MB, 10-15 fps)
- Fallback to static images when animation not possible

## Best Practices

1. **Keep it simple**: Animate ONE concept per GIF
2. **Loop seamlessly**: End state should transition smoothly to start
3. **Use consistent timing**: 2-4 seconds per concept step
4. **Label key elements**: Include text labels in the animation
5. **Test on mobile**: Ensure GIF loads quickly on slower connections
6. **Provide static fallback**: Always have a static image alternative
