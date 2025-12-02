"""Educational Content Generator Agent."""
from typing import Any, Dict, List, Optional
from dataclasses import dataclass

from ..shared.base_agent import BaseAgent, AgentResponse, AgentTool
from ..shared.config import DIAGRAM_TYPES
from .tools import DiagramGenerator, GifGenerator, SummaryGenerator, MindMapGenerator


@dataclass
class ContentRequest:
    """Request for educational content generation."""
    content_type: str  # diagram, gif, summary, mindmap
    concept: Optional[str] = None
    content: Optional[str] = None
    diagram_type: Optional[str] = None
    steps: Optional[List[str]] = None
    custom_instructions: Optional[str] = None


class EducationalContentAgent(BaseAgent):
    """Agent specialized in generating educational content.

    Capabilities:
    - Generate diagrams (8 types: concept map, flowchart, mind map, etc.)
    - Create animated GIFs for step-by-step processes
    - Generate summaries with key points
    - Create mind map structures

    Based on research from:
    - Graphics for Learning (Ruth Clark & Chopeta Lyons)
    - Educational Voice animated infographics guidelines
    - Cognitive Load Theory (Sweller)
    """

    def __init__(self, cache_dir: Optional[str] = None):
        # Initialize tools
        self.diagram_generator = DiagramGenerator(cache_dir)
        self.gif_generator = GifGenerator(cache_dir)
        self.summary_generator = SummaryGenerator()
        self.mindmap_generator = MindMapGenerator()

        # Create tool definitions
        tools = [
            AgentTool(
                name="generate_diagram",
                description="Generate an educational diagram for a concept",
                parameters={
                    "concept": {"type": "string", "description": "The concept to visualize"},
                    "diagram_type": {
                        "type": "string",
                        "enum": list(DIAGRAM_TYPES.keys()),
                        "description": "Type of diagram to generate"
                    },
                    "custom_instructions": {"type": "string", "description": "Additional instructions"}
                },
                execute=self._generate_diagram,
                required_params=["concept"]
            ),
            AgentTool(
                name="generate_gif",
                description="Generate an animated GIF showing a step-by-step process",
                parameters={
                    "title": {"type": "string", "description": "Title of the animation"},
                    "steps": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": "List of steps to animate"
                    }
                },
                execute=self._generate_gif,
                required_params=["title", "steps"]
            ),
            AgentTool(
                name="generate_summary",
                description="Generate a structured summary of content",
                parameters={
                    "content": {"type": "string", "description": "Content to summarize"},
                    "max_words": {"type": "integer", "description": "Maximum words in summary"}
                },
                execute=self._generate_summary,
                required_params=["content"]
            ),
            AgentTool(
                name="generate_mindmap",
                description="Generate a mind map structure from content",
                parameters={
                    "content": {"type": "string", "description": "Content to analyze"},
                    "central_topic": {"type": "string", "description": "Central topic for mind map"}
                },
                execute=self._generate_mindmap,
                required_params=["content"]
            )
        ]

        super().__init__(
            name="Educational Content Agent",
            description="Generates pedagogically-sound educational materials including diagrams, GIFs, summaries, and mind maps",
            tools=tools,
            system_prompt=self._get_system_prompt()
        )

    def _get_system_prompt(self) -> str:
        return """You are an Educational Content Generator Agent specialized in creating
pedagogically-sound learning materials.

Your capabilities:
1. DIAGRAMS: Generate 8 types of educational diagrams
   - concept_map: Shows relationships between concepts
   - flowchart: Sequential processes with decisions
   - mind_map: Hierarchical branching
   - architecture: System components
   - sequence: Time-ordered interactions
   - comparison: Side-by-side features
   - tree: Parent-child hierarchies
   - cycle: Circular processes

2. ANIMATED GIFs: Create step-by-step animations
   - 4-6 steps maximum (cognitive load theory)
   - 10-20 seconds total duration
   - Clear visual hierarchy
   - Progress indicators

3. SUMMARIES: Generate structured text summaries
   - 200-400 words
   - Key points extraction
   - Main concepts identification

4. MIND MAPS: Create interactive concept visualizations
   - 2-3 hierarchy levels
   - 5-15 nodes
   - Clear relationships

Design Guidelines (Research-Based):
- Visual hierarchy: Primary elements first
- Color coding: Green=#76b900, Blue=#4285f4, Orange=#ff9800
- Accessibility: 4.5:1 contrast, 14px+ fonts
- Chunking: Max 8 elements per diagram

When asked to create content:
1. Identify the appropriate content type
2. Use the corresponding tool
3. Apply educational best practices
4. Return the result with metadata
"""

    async def _generate_diagram(
        self,
        concept: str,
        diagram_type: str = "concept_map",
        custom_instructions: Optional[str] = None
    ) -> Dict[str, Any]:
        """Generate a diagram."""
        result = await self.diagram_generator.generate(
            concept=concept,
            diagram_type=diagram_type,
            custom_instructions=custom_instructions
        )
        return {
            "success": result.success,
            "url": result.url,
            "title": result.title,
            "diagram_type": result.diagram_type,
            "cached": result.cached,
            "error": result.error
        }

    async def _generate_gif(
        self,
        title: str,
        steps: List[str]
    ) -> Dict[str, Any]:
        """Generate an animated GIF."""
        result = await self.gif_generator.generate(
            title=title,
            steps=steps
        )
        return {
            "success": result.success,
            "url": result.url,
            "frames": result.frames,
            "step_count": result.step_count,
            "duration_seconds": result.duration_seconds,
            "error": result.error
        }

    async def _generate_summary(
        self,
        content: str,
        max_words: int = 400
    ) -> Dict[str, Any]:
        """Generate a summary."""
        result = await self.summary_generator.generate(
            content=content,
            max_words=max_words
        )
        return {
            "success": result.success,
            "summary": result.summary,
            "key_points": result.key_points,
            "main_concepts": result.main_concepts,
            "word_count": result.word_count,
            "error": result.error
        }

    async def _generate_mindmap(
        self,
        content: str,
        central_topic: Optional[str] = None
    ) -> Dict[str, Any]:
        """Generate a mind map."""
        result = await self.mindmap_generator.generate(
            content=content,
            central_topic=central_topic
        )
        return {
            "success": result.success,
            "nodes": result.nodes,
            "edges": result.edges,
            "central_topic": result.central_topic,
            "error": result.error
        }

    async def process(self, request: ContentRequest) -> AgentResponse:
        """Process a content generation request."""
        try:
            if request.content_type == "diagram":
                if not request.concept:
                    return AgentResponse(success=False, error="Concept required for diagram")
                result = await self._generate_diagram(
                    concept=request.concept,
                    diagram_type=request.diagram_type or "concept_map",
                    custom_instructions=request.custom_instructions
                )

            elif request.content_type == "gif":
                if not request.concept or not request.steps:
                    return AgentResponse(success=False, error="Title and steps required for GIF")
                result = await self._generate_gif(
                    title=request.concept,
                    steps=request.steps
                )

            elif request.content_type == "summary":
                if not request.content:
                    return AgentResponse(success=False, error="Content required for summary")
                result = await self._generate_summary(content=request.content)

            elif request.content_type == "mindmap":
                if not request.content:
                    return AgentResponse(success=False, error="Content required for mind map")
                result = await self._generate_mindmap(
                    content=request.content,
                    central_topic=request.concept
                )

            else:
                return AgentResponse(
                    success=False,
                    error=f"Unknown content type: {request.content_type}"
                )

            return AgentResponse(
                success=result.get("success", False),
                data=result,
                error=result.get("error")
            )

        except Exception as e:
            return AgentResponse(success=False, error=str(e))

    def get_capabilities(self) -> List[str]:
        """Return list of agent capabilities."""
        return [
            "generate_diagram",
            "generate_gif",
            "generate_summary",
            "generate_mindmap"
        ]

    def get_diagram_types(self) -> Dict[str, Dict]:
        """Get available diagram types."""
        return DIAGRAM_TYPES
