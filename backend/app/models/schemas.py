"""Pydantic schemas for API request/response models."""
from pydantic import BaseModel, Field
from typing import List, Optional


class ChatRequest(BaseModel):
    """Chat request from user."""

    message: str = Field(..., min_length=1, max_length=1000, description="User's question")
    history: List[dict] = Field(default_factory=list, description="Chat history")


class Source(BaseModel):
    """Source citation for RAG response."""

    chapter: str = Field(..., description="Chapter title")
    section: str = Field(..., description="Section heading")
    content: str = Field(..., description="Relevant content snippet")
    url: str = Field(..., description="Link to the section")
    relevance_score: float = Field(..., description="Similarity score")


class ImageResult(BaseModel):
    """Image search result."""

    id: str = Field(..., description="Image identifier")
    url: str = Field(..., description="Image URL")
    title: str = Field(..., description="Image title")
    alt_text: str = Field(..., description="Alt text for accessibility")
    chapter: str = Field(..., description="Chapter containing the image")
    section: str = Field(..., description="Section containing the image")
    score: float = Field(..., description="Relevance score")


class ChatResponse(BaseModel):
    """Chat response with sources."""

    answer: str = Field(..., description="AI-generated answer")
    sources: List[Source] = Field(default_factory=list, description="Source citations")
    images: List[ImageResult] = Field(default_factory=list, description="Relevant images")
    is_grounded: bool = Field(True, description="Whether answer is from book content")


class SearchRequest(BaseModel):
    """Search request."""

    query: str = Field(..., min_length=1, max_length=500, description="Search query")
    limit: int = Field(5, ge=1, le=20, description="Number of results")


class SearchResult(BaseModel):
    """Single search result."""

    chapter: str
    section: str
    content: str
    url: str
    score: float


class SearchResponse(BaseModel):
    """Search response."""

    results: List[SearchResult]
    query: str


class HealthResponse(BaseModel):
    """Health check response."""

    status: str = "healthy"
    qdrant_connected: bool
    openai_connected: bool
    indexed_chunks: int
