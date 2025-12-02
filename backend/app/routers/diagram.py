"""Diagram generation API endpoints.

Educational Diagram Generation API with research-based best practices:
- Concept Maps, Flowcharts, Mind Maps, Architecture Diagrams
- Animated GIF workflows for step-by-step learning
- Color-coded visual hierarchy for better comprehension
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List

from app.services.diagram import (
    get_diagram_service,
    DIAGRAM_PROMPTS,
    GIF_PROMPTS,
    DIAGRAM_TYPES,
    DIAGRAM_GUIDELINES
)


router = APIRouter(prefix="/diagram", tags=["diagram"])


class DiagramRequest(BaseModel):
    """Request to generate a diagram."""
    concept: str = Field(..., description="Concept ID (predefined) or concept name")
    custom_prompt: Optional[str] = Field(None, description="Custom prompt for generation")
    force_regenerate: bool = Field(False, description="Force regeneration even if cached")


class DiagramResponse(BaseModel):
    """Response with generated diagram info."""
    success: bool
    url: Optional[str] = None
    title: Optional[str] = None
    chapter: Optional[str] = None
    section: Optional[str] = None
    cached: bool = False
    error: Optional[str] = None


class PredefinedDiagram(BaseModel):
    """A predefined diagram concept."""
    id: str
    title: str
    diagram_type: Optional[str] = None
    chapter: str
    section: str
    learning_objective: Optional[str] = None


class DiagramTypeInfo(BaseModel):
    """Information about a diagram type."""
    type_id: str
    description: str
    best_for: List[str]
    style: str


class GifRequest(BaseModel):
    """Request to generate an animated workflow GIF."""
    workflow: str = Field(..., description="Workflow ID (predefined) or workflow name")
    custom_steps: Optional[List[str]] = Field(None, description="Custom steps for generation")
    force_regenerate: bool = Field(False, description="Force regeneration even if cached")


class GifFrame(BaseModel):
    """A single frame in the GIF animation."""
    step: int
    description: str
    url: Optional[str] = None


class GifResponse(BaseModel):
    """Response with generated GIF info."""
    success: bool
    frames: List[GifFrame] = []
    title: Optional[str] = None
    chapter: Optional[str] = None
    section: Optional[str] = None
    step_count: int = 0
    animation_only: bool = False
    cached: bool = False
    error: Optional[str] = None


class PredefinedGif(BaseModel):
    """A predefined GIF/animation workflow."""
    id: str
    title: str
    animation_type: Optional[str] = None
    chapter: str
    section: str
    step_count: int
    learning_objective: Optional[str] = None
    duration_per_step_ms: Optional[int] = 2500


@router.post("/generate", response_model=DiagramResponse)
async def generate_diagram(request: DiagramRequest) -> DiagramResponse:
    """
    Generate a diagram for a concept.

    - **concept**: Either a predefined concept ID (e.g., 'ros2-pubsub') or custom concept name
    - **custom_prompt**: Optional custom prompt for custom concepts
    - **force_regenerate**: Set to true to regenerate even if cached
    """
    service = get_diagram_service()
    result = await service.generate_diagram(
        concept=request.concept,
        custom_prompt=request.custom_prompt,
        force_regenerate=request.force_regenerate
    )
    return DiagramResponse(**result)


@router.get("/predefined", response_model=List[PredefinedDiagram])
async def list_predefined_diagrams() -> List[PredefinedDiagram]:
    """
    List all predefined diagram concepts that can be generated.

    Each diagram includes:
    - **type**: Diagram type (concept_map, flowchart, mind_map, etc.)
    - **learning_objective**: What students will understand from this diagram
    """
    diagrams = []
    for concept_id, info in DIAGRAM_PROMPTS.items():
        diagrams.append(PredefinedDiagram(
            id=concept_id,
            title=info["title"],
            diagram_type=info.get("type"),
            chapter=info.get("chapter", "general"),
            section=info.get("section", ""),
            learning_objective=info.get("learning_objective")
        ))
    return diagrams


@router.get("/types", response_model=List[DiagramTypeInfo])
async def list_diagram_types() -> List[DiagramTypeInfo]:
    """
    List all available diagram types with their best use cases.

    Educational diagram types based on research:
    - **concept_map**: Shows relationships between interconnected concepts
    - **flowchart**: Sequential processes with decision points
    - **mind_map**: Hierarchical branching from central concept
    - **architecture**: Layered system components
    - **sequence**: Time-ordered interactions
    - **comparison**: Side-by-side feature comparison
    - **tree**: Hierarchical parent-child relationships
    - **cycle**: Circular/iterative processes
    """
    types = []
    for type_id, info in DIAGRAM_TYPES.items():
        types.append(DiagramTypeInfo(
            type_id=type_id,
            description=info["description"],
            best_for=info["best_for"],
            style=info["style"]
        ))
    return types


@router.get("/guidelines")
async def get_design_guidelines() -> Dict[str, Any]:
    """
    Get educational diagram design guidelines based on research.

    Returns best practices for:
    - Visual hierarchy
    - Color schemes
    - Animation timing
    - Accessibility requirements
    """
    return {
        "guidelines": DIAGRAM_GUIDELINES,
        "sources": [
            "Graphics for Learning (Ruth Clark & Chopeta Lyons)",
            "Visible Learning research (John Hattie)",
            "Educational Voice animated infographics guidelines",
            "UAF Center for Teaching and Learning GIF guidelines"
        ]
    }


@router.get("/cached")
async def list_cached_diagrams() -> Dict[str, Any]:
    """
    List all cached/generated diagrams.
    """
    service = get_diagram_service()
    return {
        "diagrams": service.get_cached_diagrams(),
        "count": len(service.get_cached_diagrams())
    }


@router.get("/concept/{concept_id}")
async def get_diagram_info(concept_id: str) -> Dict[str, Any]:
    """
    Get information about a specific predefined diagram concept.
    """
    if concept_id not in DIAGRAM_PROMPTS:
        raise HTTPException(status_code=404, detail=f"Concept '{concept_id}' not found")

    info = DIAGRAM_PROMPTS[concept_id]
    service = get_diagram_service()

    # Check if already generated
    cache_key = service._get_cache_key(info["prompt"])
    cached = service.cache.get(cache_key)

    return {
        "id": concept_id,
        "title": info["title"],
        "chapter": info.get("chapter", "general"),
        "section": info.get("section", ""),
        "generated": cached is not None,
        "url": cached["url"] if cached else None
    }


# ============= GIF/Animation Endpoints =============

@router.post("/gif/generate", response_model=GifResponse)
async def generate_gif(request: GifRequest) -> GifResponse:
    """
    Generate an animated workflow GIF.

    - **workflow**: Either a predefined workflow ID (e.g., 'ros2-message-flow') or custom workflow name
    - **custom_steps**: Optional list of custom steps for custom workflows
    - **force_regenerate**: Set to true to regenerate even if cached
    """
    service = get_diagram_service()
    result = await service.generate_gif(
        workflow=request.workflow,
        custom_steps=request.custom_steps,
        force_regenerate=request.force_regenerate
    )
    return GifResponse(**result)


@router.get("/gif/predefined", response_model=List[PredefinedGif])
async def list_predefined_gifs() -> List[PredefinedGif]:
    """
    List all predefined GIF/animation workflows that can be generated.

    Educational GIF best practices (10-20 seconds, 4-6 steps):
    - **sequence_animation**: Linear step-by-step process
    - **cycle_animation**: Circular/repeating process
    - **request_response**: Two-way communication
    - **multimodal_flow**: Multiple inputs converging

    Each GIF includes timing recommendations and color coding.
    """
    gifs = []
    for workflow_id, info in GIF_PROMPTS.items():
        gifs.append(PredefinedGif(
            id=workflow_id,
            title=info["title"],
            animation_type=info.get("type"),
            chapter=info.get("chapter", "general"),
            section=info.get("section", ""),
            step_count=len(info.get("steps", [])),
            learning_objective=info.get("learning_objective"),
            duration_per_step_ms=info.get("duration_per_step_ms", 2500)
        ))
    return gifs


@router.get("/gif/workflow/{workflow_id}")
async def get_gif_info(workflow_id: str) -> Dict[str, Any]:
    """
    Get information about a specific predefined GIF workflow.
    """
    if workflow_id not in GIF_PROMPTS:
        raise HTTPException(status_code=404, detail=f"Workflow '{workflow_id}' not found")

    info = GIF_PROMPTS[workflow_id]
    service = get_diagram_service()

    # Check if already generated
    steps = info.get("steps", [])
    cache_key = service._get_cache_key(workflow_id + "".join(steps) + "_gif")
    cached = service.cache.get(cache_key)

    return {
        "id": workflow_id,
        "title": info["title"],
        "chapter": info.get("chapter", "general"),
        "section": info.get("section", ""),
        "steps": steps,
        "step_count": len(steps),
        "generated": cached is not None,
        "frames": cached.get("frames") if cached else None
    }
