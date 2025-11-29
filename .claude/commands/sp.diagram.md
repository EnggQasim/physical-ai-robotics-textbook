---
description: Generate conceptual diagrams using Google Gemini API for visualizing complex concepts in the textbook.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

This skill generates SVG diagrams using Google Gemini API to visualize complex concepts for the Physical AI textbook.

### 1. Parse User Input

Extract from the user input:
- **concept**: The concept to visualize (required)
- **style**: Diagram style - one of: technical, workflow, architecture, animated, simple (default: technical)
- **output**: Output filename (default: auto-generated from concept)
- **module**: Target module for the diagram (module-1-ros2, module-2-simulation, module-3-nvidia-isaac, module-4-vla)

**Examples:**
- `/sp.diagram ROS2 publisher-subscriber pattern` → technical diagram of pub/sub
- `/sp.diagram --style workflow URDF loading process` → workflow diagram
- `/sp.diagram --module module-3-nvidia-isaac Isaac Sim architecture` → architecture diagram

### 2. Validate Environment

Check that required dependencies are installed:

```bash
pip install google-generativeai pillow 2>/dev/null || pip install google-generativeai pillow
```

### 3. Determine Output Path

Based on the module parameter or auto-detect from concept:
- ROS2 concepts → `frontend/static/img/generated/ros2/`
- Simulation concepts → `frontend/static/img/generated/simulation/`
- Isaac concepts → `frontend/static/img/generated/isaac/`
- VLA concepts → `frontend/static/img/generated/vla/`
- General → `frontend/static/img/generated/`

Create the directory if it doesn't exist.

### 4. Generate Diagram

Run the Gemini image generation script:

```bash
python .specify/scripts/python/gemini_image.py \
  --prompt "<concept>" \
  --output "<output_path>" \
  --style "<style>" \
  --json
```

### 5. Handle Result

**On Success:**
1. Display the generated SVG path
2. Show a preview of the SVG (first 20 lines)
3. Generate markdown snippet for including in documentation:
   ```markdown
   ![<concept>](/<relative_path>)
   *Figure: <concept description>*
   ```
4. Suggest alt text for accessibility

**On Error:**
1. Display error message
2. Offer to generate ASCII diagram as fallback
3. If Gemini API fails, create a placeholder SVG with the concept name

### 6. Integration Suggestions

After generating the diagram, suggest:
1. Which chapter/section would benefit from this diagram
2. Caption text for the figure
3. Related concepts that might need diagrams

## Style Guide

| Style | Use Case | Example |
|-------|----------|---------|
| `technical` | System components, data structures | ROS2 node architecture |
| `workflow` | Step-by-step processes | URDF loading sequence |
| `architecture` | High-level system design | Isaac Sim layers |
| `animated` | Motion or state changes | Robot arm kinematics |
| `simple` | Basic introductory concepts | Pub/sub pattern |

## Constitution Alignment

This skill implements **Principle VIII: Visual Learning with AI (Gemini Nano)** from the project constitution:
- Generated visuals include alt text suggestions
- Stored in `/static/img/generated/` directory
- Follows NVIDIA green theme (#76b900)
- Optimized for web (SVG format is scalable and small)

## Example Workflows

### Generate ROS2 Diagram
```
/sp.diagram ROS2 topic communication between nodes
```

### Generate Architecture Diagram
```
/sp.diagram --style architecture --module module-3-nvidia-isaac NVIDIA Isaac platform components
```

### Generate Workflow
```
/sp.diagram --style workflow Gazebo simulation launch sequence
```
