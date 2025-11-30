"""Diagram generation API endpoints."""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List

from app.services.diagram import get_diagram_service, DIAGRAM_PROMPTS


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
    chapter: str
    section: str


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
    """
    diagrams = []
    for concept_id, info in DIAGRAM_PROMPTS.items():
        diagrams.append(PredefinedDiagram(
            id=concept_id,
            title=info["title"],
            chapter=info.get("chapter", "general"),
            section=info.get("section", "")
        ))
    return diagrams


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
