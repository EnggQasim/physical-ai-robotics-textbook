"""Multimodal RAG Agent."""
from typing import Any, Dict, List, Optional
from dataclasses import dataclass

from ..shared.base_agent import BaseAgent, AgentResponse, AgentTool
from ..shared.config import get_settings
from .tools import (
    TextEmbedder,
    ImageEmbedder,
    VectorSearch,
    ResponseGenerator,
    SessionManager,
    ChatMessage,
    SearchResult,
)


@dataclass
class ChatRequest:
    """Request for chat interaction."""
    message: str
    session_id: Optional[str] = None
    selected_text: Optional[str] = None
    search_images: bool = True


@dataclass
class IndexRequest:
    """Request to index content."""
    content: str
    content_type: str  # text or image
    metadata: Dict[str, Any] = None


class MultimodalRAGAgent(BaseAgent):
    """Agent specialized in Retrieval-Augmented Generation.

    Capabilities:
    - Answer questions from indexed content with citations
    - Search for text and images
    - Maintain conversation context
    - Support hybrid (text + image) search

    Architecture:
    - Dual embeddings: OpenAI text (1536d) + CLIP image (512d)
    - Vector store: Qdrant
    - Response generation: OpenAI GPT-4o-mini
    """

    def __init__(self, domain_topics: Optional[List[str]] = None):
        settings = get_settings()

        # Initialize tools
        self.text_embedder = TextEmbedder()
        self.image_embedder = ImageEmbedder()
        self.vector_search = VectorSearch()
        self.response_generator = ResponseGenerator()
        self.session_manager = SessionManager()

        self.domain_topics = domain_topics or settings.domain_topics

        # Configuration
        self.text_top_k = 5
        self.image_top_k = 3
        self.score_threshold = 0.7
        self.hybrid_weights = {"text": 0.7, "image": 0.3}

        # Create tool definitions
        tools = [
            AgentTool(
                name="search_text",
                description="Search for text content in the knowledge base",
                parameters={
                    "query": {"type": "string", "description": "Search query"},
                    "limit": {"type": "integer", "description": "Max results"}
                },
                execute=self._search_text,
                required_params=["query"]
            ),
            AgentTool(
                name="search_images",
                description="Search for images and diagrams",
                parameters={
                    "query": {"type": "string", "description": "Search query"},
                    "limit": {"type": "integer", "description": "Max results"}
                },
                execute=self._search_images,
                required_params=["query"]
            ),
            AgentTool(
                name="chat",
                description="Answer a question using RAG",
                parameters={
                    "message": {"type": "string", "description": "User message"},
                    "session_id": {"type": "string", "description": "Session ID"}
                },
                execute=self._chat,
                required_params=["message"]
            ),
            AgentTool(
                name="index_content",
                description="Index new content into the knowledge base",
                parameters={
                    "content": {"type": "string", "description": "Content to index"},
                    "metadata": {"type": "object", "description": "Content metadata"}
                },
                execute=self._index_content,
                required_params=["content"]
            )
        ]

        super().__init__(
            name="Multimodal RAG Agent",
            description="Retrieval-Augmented Generation agent supporting text and image search with conversation context",
            tools=tools,
            system_prompt=self._get_system_prompt()
        )

    def _get_system_prompt(self) -> str:
        topics = ", ".join(self.domain_topics) if self.domain_topics else "the knowledge base"
        return f"""You are a Multimodal RAG Agent specialized in answering questions
from indexed content about {topics}.

Your capabilities:
1. TEXT SEARCH: Find relevant text passages
   - Uses OpenAI embeddings (1536 dimensions)
   - Returns top-k most similar chunks

2. IMAGE SEARCH: Find relevant diagrams and images
   - Uses CLIP embeddings (512 dimensions)
   - Searches by description or visual similarity

3. HYBRID SEARCH: Combined text + image search
   - Weighted combination (70% text, 30% image)
   - Best for comprehensive answers

4. CONVERSATION: Maintain context across turns
   - Remembers last 5 messages
   - Session-based memory

Rules:
1. Only answer using retrieved context
2. Always cite sources: [Chapter X, Section Y]
3. If no relevant content found, say so clearly
4. Include relevant images when available
5. Keep responses concise but complete

Available topics: {topics}
"""

    async def _detect_query_type(self, query: str) -> str:
        """Detect if query is for text, image, or hybrid search."""
        image_keywords = [
            "diagram", "image", "picture", "show me", "visualize",
            "illustration", "figure", "chart", "graph", "screenshot"
        ]
        query_lower = query.lower()

        has_image_intent = any(kw in query_lower for kw in image_keywords)

        if has_image_intent:
            # Check if also asking for explanation (hybrid)
            explanation_keywords = ["explain", "what", "how", "why", "describe"]
            if any(kw in query_lower for kw in explanation_keywords):
                return "hybrid"
            return "image"
        return "text"

    async def _search_text(
        self,
        query: str,
        limit: int = 5
    ) -> List[Dict[str, Any]]:
        """Search for text content."""
        embedding = await self.text_embedder.embed(query)
        if not embedding:
            return []

        results = await self.vector_search.search_text(
            embedding=embedding,
            limit=limit,
            score_threshold=self.score_threshold
        )

        return [
            {
                "id": r.id,
                "content": r.content,
                "score": r.score,
                "chapter_id": r.metadata.get("chapter_id"),
                "section": r.metadata.get("section"),
                "url": r.metadata.get("url")
            }
            for r in results
        ]

    async def _search_images(
        self,
        query: str,
        limit: int = 3
    ) -> List[Dict[str, Any]]:
        """Search for images."""
        embedding = await self.image_embedder.embed_text_for_image_search(query)
        if not embedding:
            return []

        results = await self.vector_search.search_image(
            embedding=embedding,
            limit=limit,
            score_threshold=self.score_threshold
        )

        return [
            {
                "id": r.id,
                "alt_text": r.content,
                "score": r.score,
                "url": r.metadata.get("url"),
                "caption": r.metadata.get("caption"),
                "chapter_id": r.metadata.get("chapter_id")
            }
            for r in results
        ]

    async def _hybrid_search(
        self,
        query: str
    ) -> List[SearchResult]:
        """Perform hybrid text + image search."""
        results = []

        # Text search
        text_embedding = await self.text_embedder.embed(query)
        if text_embedding:
            text_results = await self.vector_search.search_text(
                embedding=text_embedding,
                limit=self.text_top_k,
                score_threshold=self.score_threshold
            )
            # Weight scores
            for r in text_results:
                r.score *= self.hybrid_weights["text"]
            results.extend(text_results)

        # Image search
        image_embedding = await self.image_embedder.embed_text_for_image_search(query)
        if image_embedding:
            image_results = await self.vector_search.search_image(
                embedding=image_embedding,
                limit=self.image_top_k,
                score_threshold=self.score_threshold
            )
            # Weight scores
            for r in image_results:
                r.score *= self.hybrid_weights["image"]
            results.extend(image_results)

        # Sort by score and dedupe
        seen_ids = set()
        unique_results = []
        for r in sorted(results, key=lambda x: x.score, reverse=True):
            if r.id not in seen_ids:
                seen_ids.add(r.id)
                unique_results.append(r)

        return unique_results[:self.text_top_k + self.image_top_k]

    async def _chat(
        self,
        message: str,
        session_id: Optional[str] = None,
        selected_text: Optional[str] = None,
        search_images: bool = True
    ) -> Dict[str, Any]:
        """Process a chat message with RAG."""
        # Get or create session
        session = self.session_manager.get_or_create_session(session_id)

        # Build query
        query = message
        if selected_text:
            query = f"Context: {selected_text}\n\nQuestion: {message}"

        # Detect query type and search
        query_type = await self._detect_query_type(query) if search_images else "text"

        if query_type == "hybrid":
            context = await self._hybrid_search(query)
        elif query_type == "image":
            image_results = await self._search_images(query)
            context = [
                SearchResult(
                    id=r["id"],
                    content=r.get("alt_text", ""),
                    content_type="image",
                    score=r["score"],
                    metadata=r
                )
                for r in image_results
            ]
        else:
            text_results = await self._search_text(query)
            context = [
                SearchResult(
                    id=r["id"],
                    content=r["content"],
                    content_type="text",
                    score=r["score"],
                    metadata=r
                )
                for r in text_results
            ]

        # Get conversation history
        history = self.session_manager.get_history(session.session_id)

        # Generate response
        response_text, citations = await self.response_generator.generate(
            query=query,
            context=context,
            history=history,
            domain_topics=self.domain_topics
        )

        # Save to session
        user_message = ChatMessage(role="user", content=message)
        assistant_message = ChatMessage(
            role="assistant",
            content=response_text,
            sources=citations
        )
        self.session_manager.add_message(session.session_id, user_message)
        self.session_manager.add_message(session.session_id, assistant_message)

        return {
            "response": response_text,
            "sources": citations,
            "session_id": session.session_id,
            "query_type": query_type
        }

    async def _index_content(
        self,
        content: str,
        content_type: str = "text",
        metadata: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Index content into the knowledge base."""
        import uuid
        chunk_id = str(uuid.uuid4())
        metadata = metadata or {}

        if content_type == "text":
            embedding = await self.text_embedder.embed(content)
            if not embedding:
                return {"success": False, "error": "Failed to generate embedding"}

            success = await self.vector_search.upsert_text(
                chunk_id=chunk_id,
                content=content,
                embedding=embedding,
                metadata=metadata
            )
        else:
            # Image - requires image path in metadata
            image_path = metadata.get("image_path")
            if not image_path:
                return {"success": False, "error": "image_path required in metadata"}

            embedding = await self.image_embedder.embed_image(image_path)
            if not embedding:
                return {"success": False, "error": "Failed to generate image embedding"}

            success = await self.vector_search.upsert_image(
                image_id=chunk_id,
                embedding=embedding,
                metadata=metadata
            )

        return {
            "success": success,
            "chunk_id": chunk_id if success else None
        }

    async def process(self, request: ChatRequest) -> AgentResponse:
        """Process a chat request."""
        try:
            result = await self._chat(
                message=request.message,
                session_id=request.session_id,
                selected_text=request.selected_text,
                search_images=request.search_images
            )

            return AgentResponse(
                success=True,
                data=result,
                metadata={
                    "session_id": result.get("session_id"),
                    "query_type": result.get("query_type"),
                    "source_count": len(result.get("sources", []))
                }
            )

        except Exception as e:
            return AgentResponse(success=False, error=str(e))

    def get_capabilities(self) -> List[str]:
        """Return list of agent capabilities."""
        return [
            "search_text",
            "search_images",
            "hybrid_search",
            "chat_with_context",
            "index_content"
        ]

    def new_session(self) -> str:
        """Create a new chat session."""
        session = self.session_manager.create_session()
        return session.session_id

    def clear_session(self, session_id: str) -> bool:
        """Clear a chat session."""
        return self.session_manager.clear_session(session_id)
