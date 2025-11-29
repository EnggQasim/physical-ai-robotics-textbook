"""Chat service with RAG for grounded responses."""
from openai import OpenAI
from typing import List, Dict, Any
from app.config import get_settings
from app.services.vector_store import get_vector_store
from app.services.image_search import get_image_search_service
from app.models.schemas import Source, ImageResult, ChatResponse


SYSTEM_PROMPT = """You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
Your role is to answer questions about robotics concepts covered in the textbook.

IMPORTANT RULES:
1. Only answer questions based on the provided context from the textbook
2. If the question is not covered in the context, say "I can only answer questions about Physical AI and Robotics topics covered in this textbook."
3. Always cite which chapter and section your answer comes from
4. Keep answers concise but informative
5. Use technical terms accurately as they appear in the textbook
6. If code examples are relevant, include them in your response

Context from the textbook:
{context}

Remember: Stay grounded in the textbook content. Do not make up information not present in the context."""


class ChatService:
    """Service for RAG-powered chat."""

    def __init__(self):
        settings = get_settings()
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.model = settings.chat_model
        self.vector_store = get_vector_store()
        self.image_search = get_image_search_service()

    def generate_response(
        self,
        message: str,
        history: List[Dict[str, str]] = None,
    ) -> ChatResponse:
        """Generate a grounded response using RAG."""
        # Search for relevant content
        search_results = self.vector_store.search(query=message, limit=5)

        # Search for relevant images
        image_results = self.image_search.search_images(query=message, limit=3)
        images = [
            ImageResult(
                id=img["id"],
                url=img["url"],
                title=img["title"],
                alt_text=img["alt_text"],
                chapter=img["chapter"],
                section=img["section"],
                score=img["score"],
            )
            for img in image_results
        ]

        # Check if we have relevant content
        if not search_results or all(r["score"] < 0.5 for r in search_results):
            # Even if no text results, return images if available
            if images:
                return ChatResponse(
                    answer="I found some relevant diagrams that might help. Here are the visual resources related to your question:",
                    sources=[],
                    images=images,
                    is_grounded=True,
                )
            return ChatResponse(
                answer="I can only answer questions about Physical AI and Robotics topics covered in this textbook. Your question doesn't seem to match the content I have available.",
                sources=[],
                images=[],
                is_grounded=False,
            )

        # Build context from search results
        context_parts = []
        sources = []
        for i, result in enumerate(search_results):
            if result["score"] >= 0.5:
                context_parts.append(
                    f"[Source {i+1}] Chapter: {result['chapter']}, Section: {result['section']}\n{result['content']}"
                )
                sources.append(
                    Source(
                        chapter=result["chapter"],
                        section=result["section"],
                        content=result["content"][:200] + "..."
                        if len(result["content"]) > 200
                        else result["content"],
                        url=result["url"],
                        relevance_score=result["score"],
                    )
                )

        context = "\n\n".join(context_parts)

        # Build messages
        messages = [{"role": "system", "content": SYSTEM_PROMPT.format(context=context)}]

        # Add history if provided
        if history:
            for msg in history[-4:]:  # Keep last 4 messages for context
                messages.append(msg)

        # Add current message
        messages.append({"role": "user", "content": message})

        # Generate response
        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=0.3,
            max_tokens=1000,
        )

        answer = response.choices[0].message.content

        return ChatResponse(
            answer=answer,
            sources=sources,
            images=images,
            is_grounded=True,
        )


# Singleton instance
_chat_service = None


def get_chat_service() -> ChatService:
    """Get or create chat service instance."""
    global _chat_service
    if _chat_service is None:
        _chat_service = ChatService()
    return _chat_service
