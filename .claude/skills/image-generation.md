---
description: Generate conceptual diagrams and icons using Google Gemini API or hand-crafted SVG for visualizing complex concepts.
---

# Image Generation Skill

This skill generates SVG diagrams and icons to visualize complex concepts for the Physical AI textbook. Supports both AI-generated (Gemini API) and hand-crafted SVG approaches.

## Capabilities

- Generate technical diagrams (system components, data structures)
- Create workflow diagrams (step-by-step processes)
- Build architecture diagrams (high-level system design)
- Simple educational diagrams (basic introductory concepts)
- **Create conceptual icons** (module icons, feature icons, navigation icons)
- **Generate logos and branding assets** (site logo, favicons)

## Usage

### Method 1: AI-Generated Diagrams (Gemini API)

When the user requests a diagram or visualization:

1. **Parse the request** to extract:
   - `concept`: What to visualize (required)
   - `style`: One of `technical`, `workflow`, `architecture`, `simple` (default: technical)
   - `module`: Target module (module-1-ros2, module-2-simulation, module-3-nvidia-isaac, module-4-vla)

2. **Generate the diagram** using the Python script:

```bash
python .specify/scripts/python/gemini_image.py \
  --prompt "<concept description>" \
  --output "frontend/static/img/generated/<filename>.svg" \
  --style "<style>" \
  --json
```

3. **Integrate into documentation**:
   - Add to relevant chapter with markdown:
   ```markdown
   ![<Alt text>](/img/generated/<filename>.svg)
   *Figure: <Caption describing the diagram>*
   ```

### Method 2: Hand-Crafted SVG Icons

For icons that need precise conceptual representation (module icons, feature icons, logos):

1. **Analyze the concept** to identify key visual elements:
   - What are the 2-3 core concepts to represent?
   - What shapes/symbols best convey meaning?
   - How can motion/flow be suggested?

2. **Create the SVG** with these specifications:
   - Canvas: 64x64 viewBox for icons, larger for logos
   - Colors: NVIDIA green `#76b900`, dark `#1a1a1a`, gray `#666`
   - Elements: Use basic shapes (circles, rects, paths, lines)
   - Optional: Add CSS animations for interactive effects

3. **Save to appropriate location**:
   - Icons: `frontend/static/img/icons/<name>-icon.svg`
   - Logos: `frontend/static/img/logo.svg`

4. **Use in React components** with `useBaseUrl`:
   ```tsx
   import useBaseUrl from '@docusaurus/useBaseUrl';
   const iconUrl = useBaseUrl('/img/icons/example-icon.svg');
   <img src={iconUrl} alt="Description" width="64" height="64" />
   ```

## Style Guide

### Diagram Styles (Gemini API)

| Style | Use Case | Example |
|-------|----------|---------|
| `technical` | System components, data structures | ROS2 node architecture |
| `workflow` | Step-by-step processes | URDF loading sequence |
| `architecture` | High-level system design | Isaac Sim layers |
| `simple` | Basic introductory concepts | Pub/sub pattern |

### Icon Design Patterns (Hand-Crafted SVG)

| Icon Type | Visual Elements | Example |
|-----------|----------------|---------|
| **Module Icon** | Core concept symbol + context | ROS2: nodes + topic + arrows |
| **Feature Icon** | Abstract representation | GPU: chip + 3D cube rendering |
| **Concept Icon** | Metaphor or analogy | AI Brain: neural network nodes |
| **Logo** | Brand identity + concept | Robot head + neural network |

### Icon Examples Created

| Icon | File | Key Visual Elements |
|------|------|---------------------|
| ROS2 Fundamentals | `ros2-icon.svg` | Publisher node → Topic → Subscriber node with data flow arrows |
| Robot Simulation | `simulation-icon.svg` | 3D cube (virtual world) + grid floor + robot inside |
| NVIDIA Isaac | `isaac-icon.svg` | GPU chip with neural network nodes + pulse effect |
| VLA Models | `vla-icon.svg` | Eye (Vision) + Speech bubble (Language) + Robot arm (Action) in cycle |
| Physical AI Intro | `intro-icon.svg` | Humanoid robot with glowing AI brain core in chest |
| Industry Tools | `industry-tools-icon.svg` | ROS2 gear + wrench + connected network nodes |
| GPU Simulation | `gpu-simulation-icon.svg` | GPU card with 3D rendering + speed lines + processing indicators |
| Cutting-Edge AI | `cutting-edge-ai-icon.svg` | Brain with neural connections + animated thinking pulse + sparkle |

## Output Paths

### Diagrams (AI-Generated)
- ROS2 concepts: `frontend/static/img/generated/ros2/`
- Simulation concepts: `frontend/static/img/generated/simulation/`
- Isaac concepts: `frontend/static/img/generated/isaac/`
- VLA concepts: `frontend/static/img/generated/vla/`
- General: `frontend/static/img/generated/`

### Icons (Hand-Crafted SVG)
- Module/Feature icons: `frontend/static/img/icons/`
- Site logo: `frontend/static/img/logo.svg`
- Favicons: `frontend/static/img/favicon.ico`

## Theme Requirements

All generated diagrams use:
- Primary accent: `#76b900` (NVIDIA green)
- Dark color: `#1a1a1a`
- Light color: `#ffffff`
- Gray: `#666666`
- Font: Arial, sans-serif

## Example Prompts

### Diagrams (AI-Generated)

1. **ROS2 Diagram**:
   ```
   Generate a diagram showing ROS2 service client-server communication
   ```

2. **Architecture Diagram**:
   ```
   Generate an architecture diagram of the NVIDIA Isaac platform components
   ```

3. **Workflow Diagram**:
   ```
   Generate a workflow showing Gazebo simulation launch sequence
   ```

### Icons (Hand-Crafted)

1. **Module Icon**:
   ```
   Create an icon for the ROS2 module showing publisher-subscriber communication
   ```

2. **Feature Icon**:
   ```
   Create an icon representing GPU-accelerated simulation with NVIDIA styling
   ```

3. **Concept Icon**:
   ```
   Create an icon for cutting-edge AI showing a brain with neural network
   ```

4. **Logo**:
   ```
   Create a site logo showing Physical AI concept - robot with AI brain
   ```

## Error Handling

If generation fails:
1. Check API quota limits
2. Try with `--model gemini-2.0-flash` flag
3. Fall back to ASCII diagram in code block

## Constitution Alignment

This skill implements **Principle VIII: Visual Learning with AI** from the project constitution:
- Generated visuals include alt text for accessibility
- Diagrams stored in `/static/img/generated/` directory
- Icons stored in `/static/img/icons/` directory
- Follows NVIDIA green theme (#76b900)
- Optimized for web (SVG format)
- Icons use `useBaseUrl` hook for proper path resolution with Docusaurus baseUrl
- Hover animations for interactive user experience
