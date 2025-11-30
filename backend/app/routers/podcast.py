"""Podcast generation API endpoints."""
from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List, Literal

from app.services.podcast import get_podcast_service, CHAPTER_PODCASTS, TTSProvider
from app.services.higgs_audio import get_higgs_audio_service, VOICE_PRESETS


router = APIRouter(prefix="/podcast", tags=["podcast"])


class PodcastGenerateRequest(BaseModel):
    """Request to generate a podcast."""
    chapter_id: str = Field(..., description="Chapter identifier")
    chapter_content: str = Field(..., description="Chapter text content")
    chapter_title: str = Field(..., description="Chapter title")
    force_regenerate: bool = Field(False, description="Force regeneration even if cached")
    tts_provider: TTSProvider = Field("openai", description="TTS provider: 'openai' or 'higgs'")
    host_voice: str = Field("en_woman", description="Voice for HOST speaker (Higgs only)")
    expert_voice: str = Field("chadwick", description="Voice for EXPERT speaker (Higgs only)")


class PodcastResponse(BaseModel):
    """Response with podcast info."""
    success: bool
    podcast_id: Optional[str] = None
    url: Optional[str] = None
    title: Optional[str] = None
    duration: Optional[int] = None  # seconds
    created_at: Optional[str] = None
    cached: bool = False
    error: Optional[str] = None
    tts_provider: Optional[str] = None


class VoiceInfo(BaseModel):
    """Info about an available voice."""
    name: str
    description: str


class TTSProvidersResponse(BaseModel):
    """Info about available TTS providers."""
    providers: List[str]
    default: str
    voices: Dict[str, str]


class ChapterPodcastInfo(BaseModel):
    """Info about a chapter's podcast availability."""
    chapter_id: str
    title: str
    description: str
    duration_estimate: int  # seconds
    has_podcast: bool
    podcast_url: Optional[str] = None


class PodcastListItem(BaseModel):
    """A podcast in the list."""
    podcast_id: str
    url: str
    title: str
    chapter_id: str
    duration: int
    created_at: str


@router.get("/providers", response_model=TTSProvidersResponse)
async def get_tts_providers() -> TTSProvidersResponse:
    """
    Get available TTS providers and voice options.

    Returns info about:
    - Available providers (openai, higgs)
    - Default provider
    - Available voices for Higgs Audio
    """
    return TTSProvidersResponse(
        providers=["openai", "higgs"],
        default="openai",
        voices=VOICE_PRESETS
    )


@router.get("/providers/higgs/status")
async def check_higgs_status() -> Dict[str, Any]:
    """
    Check if Higgs Audio HuggingFace Space is available.
    """
    higgs_service = get_higgs_audio_service()
    available = await higgs_service.check_availability()

    return {
        "provider": "higgs",
        "available": available,
        "space_url": "https://huggingface.co/spaces/smola/higgs_audio_v2",
        "note": "Higgs Audio provides multi-speaker TTS with expressive voices"
    }


@router.post("/generate", response_model=PodcastResponse)
async def generate_podcast(request: PodcastGenerateRequest) -> PodcastResponse:
    """
    Generate a podcast from chapter content.

    - **chapter_id**: Unique identifier for the chapter
    - **chapter_content**: Full text content of the chapter
    - **chapter_title**: Display title for the chapter
    - **force_regenerate**: Set to true to regenerate even if cached
    - **tts_provider**: TTS provider ('openai' or 'higgs')
    - **host_voice**: Voice for HOST speaker (Higgs only)
    - **expert_voice**: Voice for EXPERT speaker (Higgs only)
    """
    service = get_podcast_service()
    result = await service.generate_podcast(
        chapter_id=request.chapter_id,
        chapter_content=request.chapter_content,
        chapter_title=request.chapter_title,
        force_regenerate=request.force_regenerate,
        tts_provider=request.tts_provider,
        host_voice=request.host_voice,
        expert_voice=request.expert_voice
    )
    return PodcastResponse(**result)


@router.get("/chapters", response_model=List[ChapterPodcastInfo])
async def list_chapter_podcasts() -> List[ChapterPodcastInfo]:
    """
    List all chapters with podcast availability info.
    """
    service = get_podcast_service()
    cached = {p["chapter_id"]: p for p in service.get_cached_podcasts()}

    chapters = []
    for chapter_id, info in CHAPTER_PODCASTS.items():
        cached_podcast = cached.get(chapter_id)
        chapters.append(ChapterPodcastInfo(
            chapter_id=chapter_id,
            title=info["title"],
            description=info["description"],
            duration_estimate=info["duration_estimate"],
            has_podcast=cached_podcast is not None,
            podcast_url=cached_podcast["url"] if cached_podcast else None
        ))
    return chapters


@router.get("/list", response_model=List[PodcastListItem])
async def list_podcasts() -> List[PodcastListItem]:
    """
    List all generated podcasts.
    """
    service = get_podcast_service()
    podcasts = service.get_cached_podcasts()
    return [PodcastListItem(**p) for p in podcasts if "url" in p]


@router.get("/{podcast_id}", response_model=PodcastResponse)
async def get_podcast(podcast_id: str) -> PodcastResponse:
    """
    Get a specific podcast by ID.
    """
    service = get_podcast_service()
    podcast = service.get_podcast_by_id(podcast_id)

    if not podcast:
        raise HTTPException(status_code=404, detail=f"Podcast '{podcast_id}' not found")

    return PodcastResponse(**podcast)


@router.get("/chapter/{chapter_id}/info")
async def get_chapter_podcast_info(chapter_id: str) -> Dict[str, Any]:
    """
    Get podcast info for a specific chapter.
    """
    service = get_podcast_service()

    # Check if pre-defined
    predefined = service.get_chapter_podcast_info(chapter_id)

    # Check if already generated
    cached = None
    for p in service.get_cached_podcasts():
        if p.get("chapter_id") == chapter_id:
            cached = p
            break

    return {
        "chapter_id": chapter_id,
        "has_predefined_info": predefined is not None,
        "predefined_info": predefined,
        "has_generated_podcast": cached is not None,
        "podcast": cached
    }
