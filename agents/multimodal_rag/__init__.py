"""Multimodal RAG Agent.

Retrieval-Augmented Generation supporting both text and image search.
- Text-based Q&A with citations
- Image/diagram search
- Conversation context management
- Hybrid search (text + image)
"""
from .agent import MultimodalRAGAgent
from .tools import (
    TextEmbedder,
    ImageEmbedder,
    VectorSearch,
    ResponseGenerator,
)

__all__ = [
    "MultimodalRAGAgent",
    "TextEmbedder",
    "ImageEmbedder",
    "VectorSearch",
    "ResponseGenerator",
]
