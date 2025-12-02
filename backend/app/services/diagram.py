"""AI Diagram Generator Service using Google Gemini.

Educational Diagram Best Practices (Research-Based):
- Visual Hierarchy: Primary elements first, then supporting details
- Brevity: Animated GIFs should be 10-20 seconds (4-6 steps max)
- Chunking: Break complex info into manageable visual chunks
- Color Coding: Use consistent colors for concept categories
- Accessibility: 4.5:1 contrast ratio, clear labels, simple layouts
- Animation Timing: Respect information hierarchy in sequence

Diagram Types:
1. Concept Maps - Show relationships between interconnected ideas
2. Flowcharts - Sequential processes and decision paths
3. Mind Maps - Hierarchical branching from central concept
4. Architecture Diagrams - Layered system components
5. Sequence Diagrams - Time-ordered interactions
6. Comparison Diagrams - Side-by-side feature comparison
"""
import base64
import hashlib
import os
from typing import Optional, Dict, Any, List
from pathlib import Path
import json
import io
import google.generativeai as genai

from app.config import get_settings


# Educational diagram design guidelines
DIAGRAM_GUIDELINES = {
    "visual_hierarchy": "Animate/highlight primary elements first, supporting details second",
    "color_scheme": {
        "primary": "#76b900",      # NVIDIA Green - main concepts
        "secondary": "#4285f4",    # Blue - processes/actions
        "accent": "#ff9800",       # Orange - highlights/warnings
        "success": "#4caf50",      # Green - completed/success states
        "info": "#2196f3",         # Light blue - informational
        "neutral": "#757575"       # Gray - supporting elements
    },
    "animation_timing": {
        "step_duration_ms": 2500,  # Time per step (10-20 sec total for 4-8 steps)
        "transition_ms": 300,       # Transition between steps
        "max_steps": 8              # Maximum steps for comprehension
    },
    "accessibility": {
        "min_contrast_ratio": 4.5,
        "font_size_min": 14,
        "clear_labels": True
    }
}

# Diagram type templates for different educational purposes
DIAGRAM_TYPES = {
    "concept_map": {
        "description": "Shows relationships between interconnected concepts",
        "best_for": ["understanding relationships", "big picture view", "brainstorming"],
        "style": "nodes connected by labeled arrows showing relationships"
    },
    "flowchart": {
        "description": "Sequential process or workflow with decision points",
        "best_for": ["processes", "algorithms", "decision making", "troubleshooting"],
        "style": "rectangles for actions, diamonds for decisions, arrows for flow"
    },
    "mind_map": {
        "description": "Hierarchical branching from a central concept",
        "best_for": ["topic overview", "note-taking", "organizing ideas"],
        "style": "central node with radiating branches, color-coded categories"
    },
    "architecture": {
        "description": "Layered or component-based system structure",
        "best_for": ["system design", "software architecture", "hardware layout"],
        "style": "stacked layers or interconnected boxes with clear boundaries"
    },
    "sequence": {
        "description": "Time-ordered interactions between components",
        "best_for": ["communication protocols", "API calls", "message passing"],
        "style": "vertical lifelines with horizontal arrows showing messages"
    },
    "comparison": {
        "description": "Side-by-side comparison of concepts or features",
        "best_for": ["choosing between options", "understanding differences"],
        "style": "parallel columns or Venn diagram with clear labels"
    },
    "tree": {
        "description": "Hierarchical parent-child relationships",
        "best_for": ["classification", "organizational structure", "decision trees"],
        "style": "root at top, branches descending with clear hierarchy"
    },
    "cycle": {
        "description": "Circular/iterative process with repeating steps",
        "best_for": ["feedback loops", "life cycles", "iterative processes"],
        "style": "circular arrangement with arrows showing continuous flow"
    }
}

# Pre-defined diagram prompts for common robotics concepts (enhanced with educational best practices)
DIAGRAM_PROMPTS = {
    # === CONCEPT MAPS ===
    "ros2-ecosystem": {
        "title": "ROS2 Ecosystem Concept Map",
        "type": "concept_map",
        "prompt": """Create an educational CONCEPT MAP showing the ROS2 ecosystem.

        Central concept: 'ROS2' in the middle
        Connected concepts with labeled relationships:
        - 'Nodes' (are the building blocks of)
        - 'Topics' (enable pub/sub communication via)
        - 'Services' (provide request/response via)
        - 'Actions' (handle long tasks with)
        - 'Parameters' (configure behavior with)
        - 'Launch Files' (orchestrate startup with)

        Style: Clean white background, nodes as rounded rectangles,
        use #76b900 (green) for core concepts, #4285f4 (blue) for communication,
        labeled arrows showing relationships. Keep it simple with max 8 nodes.
        Font size minimum 14px for readability.""",
        "chapter": "module-1-ros2",
        "section": "index",
        "learning_objective": "Understand how ROS2 components relate to each other"
    },

    # === FLOWCHARTS ===
    "ros2-pubsub": {
        "title": "ROS2 Publisher-Subscriber Flow",
        "type": "flowchart",
        "prompt": """Create an educational FLOWCHART showing ROS2 pub/sub message flow.

        Steps (left to right):
        1. [Publisher Node] - rectangle with 'Create Message'
        2. [Serialize] - small rectangle
        3. [Topic] - hexagon shape labeled '/sensor_data'
        4. [Deserialize] - small rectangle
        5. [Subscriber Node] - rectangle with 'Process Message'

        Use arrows between each step showing data flow direction.
        Color: Green (#76b900) for nodes, blue (#4285f4) for topic.
        White background, clear labels, professional technical style.
        Include small icons: 📤 for publisher, 📥 for subscriber.""",
        "chapter": "module-1-ros2",
        "section": "nodes-topics",
        "learning_objective": "Understand the pub/sub communication pattern step by step"
    },
    "ros2-service": {
        "title": "ROS2 Service Call Flowchart",
        "type": "flowchart",
        "prompt": """Create an educational FLOWCHART showing ROS2 service communication.

        Flow (left to right with return):
        1. [Client] sends REQUEST arrow →
        2. [Service Server] receives and processes
        3. [Service Server] sends RESPONSE arrow ←
        4. [Client] receives response

        Show synchronous (blocking) nature with dotted waiting line.
        Color: Blue (#4285f4) for request, green (#76b900) for response.
        Include timing indicator showing client waits for response.
        White background, clean professional style.""",
        "chapter": "module-1-ros2",
        "section": "services-actions",
        "learning_objective": "Understand synchronous request-response pattern"
    },
    "ros2-action": {
        "title": "ROS2 Action Communication Flow",
        "type": "sequence",
        "prompt": """Create an educational SEQUENCE DIAGRAM showing ROS2 action pattern.

        Two lifelines: [Action Client] and [Action Server]

        Messages in order:
        1. Goal (Client → Server) - solid arrow
        2. Goal Response (Server → Client) - solid arrow
        3. Feedback (Server → Client) - multiple dashed arrows showing progress
        4. Result (Server → Client) - solid arrow

        Show time flowing downward.
        Color: Goals in blue (#4285f4), Feedback in orange (#ff9800), Result in green (#76b900).
        Include cancel option as optional path.
        White background, clear timeline.""",
        "chapter": "module-1-ros2",
        "section": "services-actions",
        "learning_objective": "Understand asynchronous long-running task pattern"
    },

    # === ARCHITECTURE DIAGRAMS ===
    "ros2-architecture": {
        "title": "ROS2 Architecture Layers",
        "type": "architecture",
        "prompt": """Create an educational ARCHITECTURE DIAGRAM showing ROS2 stack layers.

        Layers (top to bottom, stacked):
        1. [Your Application] - Top layer, green (#76b900)
        2. [Client Libraries (rclpy/rclcpp)] - Python/C++ APIs
        3. [ROS Core (rcl)] - Common functionality
        4. [Middleware Interface (rmw)] - Abstraction layer
        5. [DDS Implementation] - Bottom layer (CycloneDDS, FastDDS)

        Show bidirectional arrows between adjacent layers.
        Include small icons for Python 🐍 and C++ ⚡ at client library layer.
        Clean stacked boxes, white background, clear layer boundaries.""",
        "chapter": "module-1-ros2",
        "section": "index",
        "learning_objective": "Understand ROS2 software architecture layers"
    },
    "gazebo-architecture": {
        "title": "Gazebo Simulation Architecture",
        "type": "architecture",
        "prompt": """Create an educational ARCHITECTURE DIAGRAM for Gazebo simulation.

        Components (arranged as interconnected boxes):
        - [Physics Engine] - center, calculates dynamics
        - [Rendering Engine] - visual output
        - [Sensor Plugins] - camera, LiDAR, IMU
        - [Robot Model (URDF/SDF)] - robot description
        - [ROS2 Bridge] - connects to ROS2 nodes
        - [World File] - environment definition

        Show data flow arrows between components.
        Color: Simulation core in blue, ROS2 in green (#76b900), sensors in orange.
        White background, clean technical style.""",
        "chapter": "module-2-simulation",
        "section": "gazebo-basics",
        "learning_objective": "Understand Gazebo simulation components and their interactions"
    },
    "isaac-platform": {
        "title": "NVIDIA Isaac Platform Architecture",
        "type": "architecture",
        "prompt": """Create an educational ARCHITECTURE DIAGRAM for NVIDIA Isaac platform.

        Three main pillars connected to GPU base:
        1. [Isaac Sim] - Photorealistic simulation, Omniverse
        2. [Isaac ROS] - GPU-accelerated ROS2 packages
        3. [Isaac SDK] - Robot development toolkit

        Base: [NVIDIA GPU + CUDA] supporting all three

        Show connections: Sim ↔ ROS ↔ SDK, all connected to GPU.
        Use NVIDIA green (#76b900) throughout.
        Include small GPU icon and robot icon.
        Modern, clean tech style, white background.""",
        "chapter": "module-3-nvidia-isaac",
        "section": "isaac-sim",
        "learning_objective": "Understand NVIDIA Isaac platform components"
    },

    # === MIND MAPS ===
    "vla-mindmap": {
        "title": "Vision-Language-Action Mind Map",
        "type": "mind_map",
        "prompt": """Create an educational MIND MAP for VLA (Vision-Language-Action) models.

        Central: 'VLA Model' in green (#76b900)

        Branch 1 - VISION (blue):
        - Camera Input
        - Image Processing
        - Feature Extraction
        - Object Detection

        Branch 2 - LANGUAGE (orange):
        - Voice Commands
        - Text Instructions
        - Natural Language Processing
        - Intent Recognition

        Branch 3 - ACTION (green):
        - Motion Planning
        - Robot Control
        - Task Execution
        - Feedback Loop

        Radial layout, color-coded branches, clear hierarchy.
        White background, connecting lines between related concepts.""",
        "chapter": "module-4-vla",
        "section": "llm-integration",
        "learning_objective": "Understand VLA model components and their relationships"
    },

    # === TREE DIAGRAMS ===
    "urdf-structure": {
        "title": "URDF Robot Structure Tree",
        "type": "tree",
        "prompt": """Create an educational TREE DIAGRAM showing URDF robot structure.

        Root: [base_link] - fixed to world

        Tree structure:
        base_link
        ├── joint1 (revolute) → link1
        │   └── joint2 (revolute) → link2
        │       └── joint3 (revolute) → end_effector
        └── camera_joint (fixed) → camera_link

        Show Links as rectangles, Joints as circles.
        Color: Links in blue, revolute joints in green, fixed joints in gray.
        Include joint axis indicators (arrows).
        Clear hierarchy flowing top to bottom.
        White background, technical style.""",
        "chapter": "module-2-simulation",
        "section": "urdf-robots",
        "learning_objective": "Understand robot kinematic chain structure"
    },

    # === CYCLE DIAGRAMS ===
    "slam-cycle": {
        "title": "SLAM Process Cycle",
        "type": "cycle",
        "prompt": """Create an educational CYCLE DIAGRAM showing SLAM process.

        Circular flow (clockwise):
        1. [Sensor Data] - LiDAR/Camera input
        2. [Feature Extraction] - identify landmarks
        3. [Data Association] - match with known features
        4. [Pose Estimation] - calculate robot position
        5. [Map Update] - add new features to map
        → Back to Sensor Data

        Show continuous loop with arrows.
        Highlight current position uncertainty shrinking over iterations.
        Color: Sensors in orange, processing in blue, output in green.
        White background, circular arrangement.""",
        "chapter": "module-2-simulation",
        "section": "gazebo-basics",
        "learning_objective": "Understand the iterative nature of SLAM"
    },

    # === COMPARISON DIAGRAMS ===
    "ros1-vs-ros2": {
        "title": "ROS1 vs ROS2 Comparison",
        "type": "comparison",
        "prompt": """Create an educational COMPARISON DIAGRAM: ROS1 vs ROS2.

        Two columns side by side:

        ROS1 (left, gray):
        - Single Master (roscore)
        - Custom middleware
        - Python 2 / C++03
        - Linux only
        - Research focus

        ROS2 (right, green #76b900):
        - No Master (DDS)
        - Standard DDS middleware
        - Python 3 / C++17
        - Cross-platform
        - Production ready

        Use checkmarks ✓ for improvements in ROS2.
        Show migration arrow from ROS1 to ROS2.
        White background, professional comparison table style.""",
        "chapter": "module-1-ros2",
        "section": "index",
        "learning_objective": "Understand key differences between ROS1 and ROS2"
    },

    # === SEQUENCE DIAGRAMS ===
    "robot-perception": {
        "title": "Robot Perception Pipeline",
        "type": "flowchart",
        "prompt": """Create an educational FLOWCHART showing robot perception pipeline.

        Linear flow (left to right):
        [Sensors] → [Preprocessing] → [Feature Extraction] → [Object Detection] → [Scene Understanding] → [Decision]

        Sensors include icons: 📷 Camera, 📡 LiDAR, 🧭 IMU

        Show parallel paths merging at Sensor Fusion step.
        Color gradient: Raw data (orange) → Processed (blue) → Decision (green).
        Include data type labels (images, point clouds, poses).
        White background, clean technical style.""",
        "chapter": "module-4-vla",
        "section": "voice-commands",
        "learning_objective": "Understand how robots process sensor data for decision making"
    }
}

# Pre-defined GIF/Animation prompts for workflow visualizations
# Best Practice: 4-6 steps, 10-20 seconds total, clear visual hierarchy
GIF_PROMPTS = {
    "ros2-message-flow": {
        "title": "ROS2 Message Flow Animation",
        "type": "sequence_animation",
        "steps": [
            "Publisher Node CREATES message with sensor data [highlight: publisher box glows green]",
            "Message is SERIALIZED into binary format [highlight: data transforms into packets]",
            "DDS Middleware ROUTES message through topic '/sensor_data' [highlight: topic channel pulses]",
            "Subscriber Node RECEIVES and deserializes message [highlight: subscriber box glows green]",
            "Callback function PROCESSES the data [highlight: processing indicator]"
        ],
        "chapter": "module-1-ros2",
        "section": "nodes-topics",
        "learning_objective": "Visualize the complete pub/sub message lifecycle",
        "duration_per_step_ms": 2500,
        "colors": {
            "active": "#76b900",
            "inactive": "#e0e0e0",
            "data_flow": "#4285f4"
        }
    },
    "ros2-service-call": {
        "title": "ROS2 Service Request-Response Animation",
        "type": "request_response",
        "steps": [
            "Client Node CREATES service request [highlight: client glows, request forms]",
            "Request travels TO server [highlight: arrow animates left-to-right]",
            "Server RECEIVES and processes request [highlight: server glows, gear spins]",
            "Server GENERATES response [highlight: response forms]",
            "Response returns TO client [highlight: arrow animates right-to-left]",
            "Client RECEIVES response [highlight: client glows green - success]"
        ],
        "chapter": "module-1-ros2",
        "section": "services-actions",
        "learning_objective": "Understand synchronous service communication timing",
        "duration_per_step_ms": 2000,
        "colors": {
            "request": "#4285f4",
            "response": "#76b900",
            "processing": "#ff9800"
        }
    },
    "ros2-action-flow": {
        "title": "ROS2 Action Server Animation",
        "type": "async_communication",
        "steps": [
            "Client sends GOAL to Action Server [highlight: goal arrow]",
            "Server ACCEPTS goal and begins execution [highlight: server starts]",
            "Server sends FEEDBACK update #1 [highlight: progress 25%]",
            "Server sends FEEDBACK update #2 [highlight: progress 50%]",
            "Server sends FEEDBACK update #3 [highlight: progress 75%]",
            "Server completes and sends RESULT [highlight: success checkmark]"
        ],
        "chapter": "module-1-ros2",
        "section": "services-actions",
        "learning_objective": "See how long-running tasks provide progress updates",
        "duration_per_step_ms": 2000,
        "colors": {
            "goal": "#4285f4",
            "feedback": "#ff9800",
            "result": "#76b900"
        }
    },
    "gazebo-physics-loop": {
        "title": "Gazebo Simulation Loop Animation",
        "type": "cycle_animation",
        "steps": [
            "READ sensor data from simulated world [highlight: sensors pulse]",
            "RECEIVE robot control commands from ROS2 [highlight: command input]",
            "CALCULATE physics: collisions, forces, gravity [highlight: physics engine]",
            "UPDATE robot and object positions [highlight: world state changes]",
            "RENDER 3D visualization [highlight: display updates]",
            "REPEAT at simulation rate (1000 Hz) [highlight: loop arrow]"
        ],
        "chapter": "module-2-simulation",
        "section": "gazebo-basics",
        "learning_objective": "Understand the continuous simulation update cycle",
        "duration_per_step_ms": 2500,
        "colors": {
            "sensors": "#ff9800",
            "physics": "#4285f4",
            "render": "#76b900"
        }
    },
    "urdf-loading": {
        "title": "URDF Robot Loading Process",
        "type": "sequence_animation",
        "steps": [
            "PARSE URDF/SDF XML file [highlight: file icon]",
            "CREATE link meshes and collision shapes [highlight: 3D shapes appear]",
            "CONNECT joints between links [highlight: joints animate]",
            "APPLY physical properties (mass, inertia) [highlight: physics labels]",
            "SPAWN robot in simulation world [highlight: robot appears]"
        ],
        "chapter": "module-2-simulation",
        "section": "urdf-robots",
        "learning_objective": "See how robot descriptions become simulated robots",
        "duration_per_step_ms": 3000,
        "colors": {
            "parsing": "#757575",
            "geometry": "#4285f4",
            "physics": "#ff9800",
            "spawn": "#76b900"
        }
    },
    "isaac-sim-workflow": {
        "title": "Isaac Sim Training Workflow",
        "type": "pipeline_animation",
        "steps": [
            "CREATE photorealistic virtual environment [highlight: world loads]",
            "SPAWN robot with sensors and actuators [highlight: robot appears]",
            "RUN training episode with random variations [highlight: robot moves]",
            "COLLECT experience data and rewards [highlight: data streams]",
            "UPDATE neural network policy [highlight: brain icon pulses]",
            "EXPORT trained model for real robot [highlight: export arrow]"
        ],
        "chapter": "module-3-nvidia-isaac",
        "section": "isaac-sim",
        "learning_objective": "Understand sim-to-real training pipeline",
        "duration_per_step_ms": 2500,
        "colors": {
            "simulation": "#76b900",
            "training": "#4285f4",
            "export": "#ff9800"
        }
    },
    "vla-inference": {
        "title": "VLA Model Inference Pipeline",
        "type": "multimodal_flow",
        "steps": [
            "CAPTURE camera image of scene [highlight: camera icon, image appears]",
            "RECORD voice command: 'Pick up the red cup' [highlight: microphone, waveform]",
            "ENCODE image through Vision Transformer [highlight: vision encoder]",
            "ENCODE text through Language Model [highlight: language encoder]",
            "FUSE multimodal features [highlight: fusion module glows]",
            "GENERATE robot action sequence [highlight: action commands output]",
            "EXECUTE motion on robot arm [highlight: robot moves]"
        ],
        "chapter": "module-4-vla",
        "section": "llm-integration",
        "learning_objective": "Trace how vision and language become robot actions",
        "duration_per_step_ms": 2000,
        "colors": {
            "vision": "#4285f4",
            "language": "#ff9800",
            "fusion": "#9c27b0",
            "action": "#76b900"
        }
    },
    "slam-loop": {
        "title": "SLAM Algorithm Cycle",
        "type": "cycle_animation",
        "steps": [
            "SENSE: LiDAR/Camera captures environment [highlight: sensor beams]",
            "EXTRACT: Identify features and landmarks [highlight: feature points]",
            "ASSOCIATE: Match features with known map [highlight: matching lines]",
            "ESTIMATE: Calculate robot pose [highlight: robot position updates]",
            "UPDATE: Add new features to map [highlight: map grows]",
            "REPEAT: Continuous localization and mapping [highlight: loop completes]"
        ],
        "chapter": "module-2-simulation",
        "section": "gazebo-basics",
        "learning_objective": "Understand the iterative SLAM process",
        "duration_per_step_ms": 2500,
        "colors": {
            "sensing": "#ff9800",
            "processing": "#4285f4",
            "mapping": "#76b900"
        }
    }
}


class DiagramService:
    """Service for generating diagrams using Google Gemini."""

    def __init__(self):
        settings = get_settings()
        if settings.gemini_api_key:
            genai.configure(api_key=settings.gemini_api_key)
            self.model = genai.GenerativeModel(settings.gemini_model)
        else:
            self.model = None

        # Cache directory for generated diagrams
        self.cache_dir = Path(__file__).parent.parent.parent.parent / "frontend" / "static" / "img" / "generated"
        self.cache_dir.mkdir(parents=True, exist_ok=True)

        # Cache metadata file
        self.cache_file = self.cache_dir / "cache.json"
        self.cache = self._load_cache()

    def _load_cache(self) -> Dict[str, Any]:
        """Load diagram cache metadata."""
        if self.cache_file.exists():
            with open(self.cache_file, "r") as f:
                return json.load(f)
        return {}

    def _save_cache(self):
        """Save diagram cache metadata."""
        with open(self.cache_file, "w") as f:
            json.dump(self.cache, f, indent=2)

    def _get_cache_key(self, prompt: str) -> str:
        """Generate a cache key from prompt."""
        return hashlib.md5(prompt.encode()).hexdigest()[:12]

    async def generate_diagram(
        self,
        concept: str,
        custom_prompt: Optional[str] = None,
        force_regenerate: bool = False
    ) -> Dict[str, Any]:
        """
        Generate a diagram for a concept.

        Args:
            concept: Pre-defined concept ID or custom concept name
            custom_prompt: Optional custom prompt for generation
            force_regenerate: Force regeneration even if cached

        Returns:
            Dict with diagram URL, title, and metadata
        """
        if not self.model:
            return {
                "success": False,
                "error": "Gemini API not configured",
                "url": None
            }

        # Get prompt from predefined or custom
        if concept in DIAGRAM_PROMPTS:
            diagram_info = DIAGRAM_PROMPTS[concept]
            prompt = diagram_info["prompt"]
            title = diagram_info["title"]
            chapter = diagram_info.get("chapter", "general")
            section = diagram_info.get("section", "")
        else:
            prompt = custom_prompt or f"Create a clear, professional technical diagram explaining the concept of {concept} in robotics and AI. Use modern, clean styling."
            title = f"{concept} Diagram"
            chapter = "custom"
            section = ""

        # Check cache
        cache_key = self._get_cache_key(prompt)
        if not force_regenerate and cache_key in self.cache:
            cached = self.cache[cache_key]
            return {
                "success": True,
                "url": cached["url"],
                "title": cached["title"],
                "chapter": cached.get("chapter", chapter),
                "section": cached.get("section", section),
                "cached": True
            }

        try:
            # Generate image using Gemini
            response = self.model.generate_content(
                [
                    prompt,
                    "Generate this as a clean, professional SVG or PNG diagram suitable for a technical textbook. Use a white background."
                ],
                generation_config=genai.types.GenerationConfig(
                    temperature=0.7,
                )
            )

            # Check if response contains image
            if response.candidates and response.candidates[0].content.parts:
                for part in response.candidates[0].content.parts:
                    if hasattr(part, 'inline_data') and part.inline_data:
                        # Save image
                        image_data = part.inline_data.data
                        mime_type = part.inline_data.mime_type

                        ext = "png" if "png" in mime_type else "jpg"
                        filename = f"{cache_key}.{ext}"
                        filepath = self.cache_dir / chapter
                        filepath.mkdir(parents=True, exist_ok=True)
                        filepath = filepath / filename

                        with open(filepath, "wb") as f:
                            f.write(base64.b64decode(image_data) if isinstance(image_data, str) else image_data)

                        # Update cache
                        url = f"/img/generated/{chapter}/{filename}"
                        self.cache[cache_key] = {
                            "url": url,
                            "title": title,
                            "chapter": chapter,
                            "section": section,
                            "prompt": prompt[:200]
                        }
                        self._save_cache()

                        return {
                            "success": True,
                            "url": url,
                            "title": title,
                            "chapter": chapter,
                            "section": section,
                            "cached": False
                        }

            # If no image generated, return text description
            return {
                "success": False,
                "error": "No image generated - Gemini returned text only",
                "text_response": response.text if hasattr(response, 'text') else str(response),
                "url": None
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "url": None
            }

    async def generate_gif(
        self,
        workflow: str,
        custom_steps: Optional[List[str]] = None,
        force_regenerate: bool = False
    ) -> Dict[str, Any]:
        """
        Generate an animated GIF for a workflow.

        Since Gemini doesn't directly generate GIFs, we generate individual
        step images and combine them with metadata for frontend animation.

        Args:
            workflow: Pre-defined workflow ID or custom workflow name
            custom_steps: Optional list of custom steps for generation
            force_regenerate: Force regeneration even if cached

        Returns:
            Dict with GIF frames/metadata, title, and info
        """
        if not self.model:
            return {
                "success": False,
                "error": "Gemini API not configured",
                "url": None
            }

        # Get steps from predefined or custom
        if workflow in GIF_PROMPTS:
            gif_info = GIF_PROMPTS[workflow]
            steps = gif_info["steps"]
            title = gif_info["title"]
            chapter = gif_info.get("chapter", "general")
            section = gif_info.get("section", "")
        else:
            steps = custom_steps or [
                f"Step 1: Initialize {workflow}",
                f"Step 2: Process {workflow}",
                f"Step 3: Complete {workflow}"
            ]
            title = f"{workflow} Workflow"
            chapter = "custom"
            section = ""

        # Generate cache key from workflow + steps
        cache_key = self._get_cache_key(workflow + "".join(steps) + "_gif")

        # Check cache
        if not force_regenerate and cache_key in self.cache:
            cached = self.cache[cache_key]
            if cached.get("type") == "gif":
                return {
                    "success": True,
                    "frames": cached["frames"],
                    "title": cached["title"],
                    "chapter": cached.get("chapter", chapter),
                    "section": cached.get("section", section),
                    "step_count": len(cached["frames"]),
                    "cached": True
                }

        frames = []

        try:
            # Generate an image for each step
            for i, step in enumerate(steps):
                step_prompt = f"""Create a clear, professional technical diagram showing:
                {step}

                This is step {i+1} of {len(steps)} in a workflow animation.
                Highlight the current active component in green (#76b900).
                Use a white background with clean, modern styling.
                Include a step indicator showing "{i+1}/{len(steps)}" in the corner.
                """

                response = self.model.generate_content(
                    [step_prompt],
                    generation_config=genai.types.GenerationConfig(
                        temperature=0.7,
                    )
                )

                # Check if response contains image
                if response.candidates and response.candidates[0].content.parts:
                    for part in response.candidates[0].content.parts:
                        if hasattr(part, 'inline_data') and part.inline_data:
                            image_data = part.inline_data.data
                            mime_type = part.inline_data.mime_type

                            ext = "png" if "png" in mime_type else "jpg"
                            filename = f"{cache_key}_step{i+1}.{ext}"
                            filepath = self.cache_dir / chapter / "gifs"
                            filepath.mkdir(parents=True, exist_ok=True)
                            filepath = filepath / filename

                            with open(filepath, "wb") as f:
                                data = base64.b64decode(image_data) if isinstance(image_data, str) else image_data
                                f.write(data)

                            url = f"/img/generated/{chapter}/gifs/{filename}"
                            frames.append({
                                "step": i + 1,
                                "description": step,
                                "url": url
                            })
                            break

            if frames:
                # Cache the GIF data
                self.cache[cache_key] = {
                    "type": "gif",
                    "frames": frames,
                    "title": title,
                    "chapter": chapter,
                    "section": section,
                    "workflow": workflow
                }
                self._save_cache()

                return {
                    "success": True,
                    "frames": frames,
                    "title": title,
                    "chapter": chapter,
                    "section": section,
                    "step_count": len(frames),
                    "cached": False
                }

            # Fallback: return step descriptions for CSS animation
            return {
                "success": True,
                "frames": [{"step": i+1, "description": s, "url": None} for i, s in enumerate(steps)],
                "title": title,
                "chapter": chapter,
                "section": section,
                "step_count": len(steps),
                "animation_only": True,
                "cached": False
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "frames": []
            }

    def get_predefined_diagrams(self) -> Dict[str, Dict]:
        """Get list of all predefined diagram concepts."""
        return DIAGRAM_PROMPTS

    def get_predefined_gifs(self) -> Dict[str, Dict]:
        """Get list of all predefined GIF/animation workflows."""
        return GIF_PROMPTS

    def get_cached_diagrams(self) -> Dict[str, Any]:
        """Get all cached/generated diagrams."""
        return self.cache


# Singleton instance
_diagram_service: Optional[DiagramService] = None


def get_diagram_service() -> DiagramService:
    """Get or create diagram service singleton."""
    global _diagram_service
    if _diagram_service is None:
        _diagram_service = DiagramService()
    return _diagram_service
