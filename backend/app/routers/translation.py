"""Translation API endpoints."""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any

from app.services.translation import get_translation_service


router = APIRouter(prefix="/translation", tags=["translation"])


# Request/Response Models

class TranslateRequest(BaseModel):
    """Request to translate text to Urdu."""
    text: str = Field(..., description="English text to translate", max_length=5000)
    preserve_technical_terms: bool = Field(
        True,
        description="Keep technical terms as 'Urdu (English)' format"
    )
    context: Optional[str] = Field(
        None,
        description="Context about the content (e.g., 'robotics', 'programming')"
    )


class TranslateResponse(BaseModel):
    """Translation response."""
    original: str
    translation: str
    from_cache: bool
    language: str = "ur"


class CacheStatsResponse(BaseModel):
    """Translation cache statistics."""
    cached_translations: int
    cache_size_bytes: int


# Endpoints

@router.post("/translate", response_model=TranslateResponse)
async def translate_text(request: TranslateRequest) -> TranslateResponse:
    """
    Translate English text to Urdu.

    Uses GPT-4o-mini for high-quality translation with:
    - Technical term preservation
    - RTL-ready Urdu output
    - Translation caching
    """
    service = get_translation_service()

    # Check cache first
    cached = service.get_cached_translation(request.text)
    if cached:
        return TranslateResponse(
            original=request.text,
            translation=cached,
            from_cache=True
        )

    try:
        translation = await service.translate_to_urdu(
            text=request.text,
            preserve_technical_terms=request.preserve_technical_terms,
            context=request.context
        )

        return TranslateResponse(
            original=request.text,
            translation=translation,
            from_cache=False
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")


@router.post("/translate/paragraph", response_model=TranslateResponse)
async def translate_paragraph(request: TranslateRequest) -> TranslateResponse:
    """
    Translate a paragraph with mixed content handling.

    Preserves code blocks and inline code while translating prose.
    """
    service = get_translation_service()

    try:
        translation = await service.translate_paragraph(
            paragraph=request.text,
            context=request.context
        )

        cached = service.get_cached_translation(request.text)

        return TranslateResponse(
            original=request.text,
            translation=translation,
            from_cache=cached is not None
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")


@router.get("/cache/stats", response_model=CacheStatsResponse)
async def get_cache_stats() -> CacheStatsResponse:
    """Get translation cache statistics."""
    service = get_translation_service()
    stats = service.get_cache_stats()
    return CacheStatsResponse(**stats)


@router.post("/cache/clear")
async def clear_cache() -> Dict[str, bool]:
    """Clear the translation cache."""
    service = get_translation_service()
    service.clear_cache()
    return {"cleared": True}
