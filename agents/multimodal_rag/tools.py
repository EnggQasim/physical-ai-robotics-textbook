"""Tools for Multimodal RAG Agent."""
import hashlib
import json
import uuid
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field

from openai import AsyncOpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models as qdrant_models

from ..shared.config import get_settings


@dataclass
class SearchResult:
    """A single search result."""
    id: str
    content: str
    content_type: str  # text or image
    score: float
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ChatMessage:
    """A chat message."""
    role: str  # user or assistant
    content: str
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())
    sources: List[Dict[str, Any]] = field(default_factory=list)


@dataclass
class ChatSession:
    """A chat session."""
    session_id: str
    messages: List[ChatMessage] = field(default_factory=list)
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())
    last_active: str = field(default_factory=lambda: datetime.now().isoformat())


class TextEmbedder:
    """Generate text embeddings using OpenAI."""

    def __init__(self):
        settings = get_settings()
        if settings.openai_api_key:
            self.client = AsyncOpenAI(api_key=settings.openai_api_key)
            self.model = settings.openai_embedding_model
            self.dimensions = settings.text_embedding_size
        else:
            self.client = None

    async def embed(self, text: str) -> Optional[List[float]]:
        """Generate embedding for text."""
        if not self.client:
            return None

        try:
            response = await self.client.embeddings.create(
                model=self.model,
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            print(f"Embedding error: {e}")
            return None

    async def embed_batch(self, texts: List[str]) -> List[Optional[List[float]]]:
        """Generate embeddings for multiple texts."""
        if not self.client:
            return [None] * len(texts)

        try:
            response = await self.client.embeddings.create(
                model=self.model,
                input=texts
            )
            return [item.embedding for item in response.data]
        except Exception as e:
            print(f"Batch embedding error: {e}")
            return [None] * len(texts)


class ImageEmbedder:
    """Generate image embeddings using CLIP (placeholder - requires transformers)."""

    def __init__(self):
        settings = get_settings()
        self.dimensions = settings.image_embedding_size
        self._model = None
        self._processor = None

    def _load_model(self):
        """Lazy load CLIP model."""
        if self._model is None:
            try:
                from transformers import CLIPProcessor, CLIPModel
                self._model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
                self._processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
            except ImportError:
                print("transformers not installed. Image embedding unavailable.")
                return False
        return True

    async def embed_image(self, image_path: str) -> Optional[List[float]]:
        """Generate embedding for an image."""
        if not self._load_model():
            return None

        try:
            from PIL import Image
            image = Image.open(image_path)
            inputs = self._processor(images=image, return_tensors="pt")
            outputs = self._model.get_image_features(**inputs)
            return outputs[0].detach().numpy().tolist()
        except Exception as e:
            print(f"Image embedding error: {e}")
            return None

    async def embed_text_for_image_search(self, text: str) -> Optional[List[float]]:
        """Generate CLIP text embedding for image search."""
        if not self._load_model():
            return None

        try:
            inputs = self._processor(text=[text], return_tensors="pt", padding=True)
            outputs = self._model.get_text_features(**inputs)
            return outputs[0].detach().numpy().tolist()
        except Exception as e:
            print(f"Text-to-image embedding error: {e}")
            return None


class VectorSearch:
    """Search vector database (Qdrant)."""

    def __init__(self):
        settings = get_settings()
        self.collection_name = settings.qdrant_collection

        if settings.qdrant_url and settings.qdrant_api_key:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key
            )
        elif settings.qdrant_url:
            self.client = QdrantClient(url=settings.qdrant_url)
        else:
            # Use in-memory client for testing
            self.client = QdrantClient(":memory:")

        self._ensure_collection()

    def _ensure_collection(self):
        """Ensure collection exists."""
        settings = get_settings()
        try:
            self.client.get_collection(self.collection_name)
        except Exception:
            # Create collection with multiple vectors
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config={
                    "text": qdrant_models.VectorParams(
                        size=settings.text_embedding_size,
                        distance=qdrant_models.Distance.COSINE
                    ),
                    "image": qdrant_models.VectorParams(
                        size=settings.image_embedding_size,
                        distance=qdrant_models.Distance.COSINE
                    )
                }
            )

    async def search_text(
        self,
        embedding: List[float],
        limit: int = 5,
        score_threshold: float = 0.7
    ) -> List[SearchResult]:
        """Search for text content."""
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=("text", embedding),
                limit=limit,
                score_threshold=score_threshold
            )
            return [
                SearchResult(
                    id=str(r.id),
                    content=r.payload.get("content", ""),
                    content_type="text",
                    score=r.score,
                    metadata={
                        "chapter_id": r.payload.get("chapter_id"),
                        "section": r.payload.get("section"),
                        "url": r.payload.get("url")
                    }
                )
                for r in results
            ]
        except Exception as e:
            print(f"Text search error: {e}")
            return []

    async def search_image(
        self,
        embedding: List[float],
        limit: int = 3,
        score_threshold: float = 0.7
    ) -> List[SearchResult]:
        """Search for image content."""
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=("image", embedding),
                limit=limit,
                score_threshold=score_threshold
            )
            return [
                SearchResult(
                    id=str(r.id),
                    content=r.payload.get("alt_text", ""),
                    content_type="image",
                    score=r.score,
                    metadata={
                        "url": r.payload.get("url"),
                        "caption": r.payload.get("caption"),
                        "chapter_id": r.payload.get("chapter_id")
                    }
                )
                for r in results
            ]
        except Exception as e:
            print(f"Image search error: {e}")
            return []

    async def upsert_text(
        self,
        chunk_id: str,
        content: str,
        embedding: List[float],
        metadata: Dict[str, Any]
    ) -> bool:
        """Insert or update text content."""
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    qdrant_models.PointStruct(
                        id=chunk_id,
                        vector={"text": embedding},
                        payload={
                            "content": content,
                            "content_type": "text",
                            **metadata
                        }
                    )
                ]
            )
            return True
        except Exception as e:
            print(f"Upsert error: {e}")
            return False

    async def upsert_image(
        self,
        image_id: str,
        embedding: List[float],
        metadata: Dict[str, Any]
    ) -> bool:
        """Insert or update image content."""
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    qdrant_models.PointStruct(
                        id=image_id,
                        vector={"image": embedding},
                        payload={
                            "content_type": "image",
                            **metadata
                        }
                    )
                ]
            )
            return True
        except Exception as e:
            print(f"Image upsert error: {e}")
            return False


class ResponseGenerator:
    """Generate responses using OpenAI."""

    def __init__(self):
        settings = get_settings()
        if settings.openai_api_key:
            self.client = AsyncOpenAI(api_key=settings.openai_api_key)
            self.model = settings.openai_chat_model
        else:
            self.client = None

    async def generate(
        self,
        query: str,
        context: List[SearchResult],
        history: List[ChatMessage],
        domain_topics: Optional[List[str]] = None
    ) -> Tuple[str, List[Dict[str, Any]]]:
        """Generate a response with citations."""
        if not self.client:
            return "API not configured", []

        # Build context string
        context_str = "\n\n".join([
            f"[Source {i+1}] ({r.metadata.get('chapter_id', 'unknown')}, {r.metadata.get('section', 'unknown')})\n{r.content}"
            for i, r in enumerate(context)
        ])

        # Build history string
        history_str = "\n".join([
            f"{m.role}: {m.content}"
            for m in history[-5:]  # Last 5 messages
        ])

        topics_str = ", ".join(domain_topics) if domain_topics else "the knowledge base"

        system_prompt = f"""You are a helpful assistant that answers questions based ONLY on the provided context.

Rules:
1. Only answer using the provided context
2. Cite sources like this: [Source 1] or [Chapter X, Section Y]
3. If information is not in context, say "I couldn't find information about that in {topics_str}"
4. Keep responses concise but complete
5. Include relevant images when available

Context:
{context_str}

Conversation History:
{history_str}
"""

        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": query}
                ]
            )

            answer = response.choices[0].message.content

            # Extract citations
            citations = [
                {
                    "source_id": i + 1,
                    "chapter_id": r.metadata.get("chapter_id"),
                    "section": r.metadata.get("section"),
                    "url": r.metadata.get("url"),
                    "relevance_score": r.score
                }
                for i, r in enumerate(context)
            ]

            return answer, citations

        except Exception as e:
            return f"Error generating response: {e}", []


class SessionManager:
    """Manage chat sessions."""

    def __init__(self):
        self._sessions: Dict[str, ChatSession] = {}
        self._max_messages = 50
        self._expiry_minutes = 30

    def create_session(self) -> ChatSession:
        """Create a new session."""
        session_id = str(uuid.uuid4())
        session = ChatSession(session_id=session_id)
        self._sessions[session_id] = session
        return session

    def get_session(self, session_id: str) -> Optional[ChatSession]:
        """Get an existing session."""
        session = self._sessions.get(session_id)
        if session:
            session.last_active = datetime.now().isoformat()
        return session

    def get_or_create_session(self, session_id: Optional[str] = None) -> ChatSession:
        """Get existing session or create new one."""
        if session_id:
            session = self.get_session(session_id)
            if session:
                return session
        return self.create_session()

    def add_message(self, session_id: str, message: ChatMessage) -> bool:
        """Add a message to a session."""
        session = self.get_session(session_id)
        if not session:
            return False

        session.messages.append(message)

        # Trim if too many messages
        if len(session.messages) > self._max_messages:
            session.messages = session.messages[-self._max_messages:]

        return True

    def get_history(self, session_id: str, limit: int = 5) -> List[ChatMessage]:
        """Get recent message history."""
        session = self.get_session(session_id)
        if not session:
            return []
        return session.messages[-limit:]

    def clear_session(self, session_id: str) -> bool:
        """Clear a session."""
        if session_id in self._sessions:
            del self._sessions[session_id]
            return True
        return False
