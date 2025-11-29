"""Image search service using text descriptions for semantic matching."""
from typing import List, Dict, Any, Optional
from app.services.embedding import get_embedding_service
import re


# Pre-defined image metadata with semantic descriptions
# These descriptions are used for semantic search instead of CLIP embeddings
IMAGE_METADATA = [
    {
        "id": "pubsub-diagram",
        "url": "/img/generated/ros2/pubsub-diagram.svg",
        "title": "ROS2 Publisher-Subscriber Pattern",
        "alt_text": "Diagram showing ROS2 publisher-subscriber communication pattern",
        "description": "ROS2 publisher subscriber communication pattern diagram showing nodes publishing and subscribing to topics. A publisher node sends messages to a topic, and subscriber nodes receive messages from the same topic. This is the core communication mechanism in ROS2 for asynchronous message passing between nodes.",
        "chapter": "ROS2 Fundamentals",
        "section": "Nodes and Topics",
        "keywords": ["ros2", "publisher", "subscriber", "topic", "node", "message", "communication"],
    },
    {
        "id": "service-diagram",
        "url": "/img/generated/ros2/service-diagram.svg",
        "title": "ROS2 Service-Client Pattern",
        "alt_text": "Diagram showing ROS2 service-client request-response pattern",
        "description": "ROS2 service client communication pattern diagram showing request-response mechanism. A client node sends a request to a service server, which processes the request and returns a response. This is used for synchronous operations in ROS2.",
        "chapter": "ROS2 Fundamentals",
        "section": "Services and Actions",
        "keywords": ["ros2", "service", "client", "request", "response", "synchronous"],
    },
    {
        "id": "gazebo-architecture",
        "url": "/img/generated/simulation/gazebo-architecture.svg",
        "title": "Gazebo Simulation Architecture",
        "alt_text": "Diagram showing Gazebo simulation platform architecture",
        "description": "Gazebo simulation architecture diagram showing the physics engine, rendering engine, sensors, and robot models. Gazebo is a powerful robot simulation tool that integrates with ROS2 for testing robot algorithms in a virtual environment.",
        "chapter": "Robot Simulation",
        "section": "Gazebo Basics",
        "keywords": ["gazebo", "simulation", "physics", "rendering", "sensors", "robot", "virtual"],
    },
    {
        "id": "isaac-platform",
        "url": "/img/generated/isaac/isaac-platform.svg",
        "title": "NVIDIA Isaac Platform Architecture",
        "alt_text": "Diagram showing NVIDIA Isaac Sim platform layers",
        "description": "NVIDIA Isaac platform architecture diagram showing Isaac Sim built on Omniverse, Isaac ROS for perception and navigation, and the GPU-accelerated simulation capabilities. Isaac enables high-fidelity robot simulation with synthetic data generation.",
        "chapter": "NVIDIA Isaac Platform",
        "section": "Isaac Sim",
        "keywords": ["nvidia", "isaac", "sim", "omniverse", "gpu", "simulation", "synthetic data"],
    },
    {
        "id": "vla-pipeline",
        "url": "/img/generated/vla/vla-pipeline.svg",
        "title": "Vision-Language-Action Pipeline",
        "alt_text": "Diagram showing VLA model pipeline from vision to action",
        "description": "Vision-Language-Action (VLA) pipeline diagram showing how visual input and language commands are processed through a multimodal model to generate robot actions. This is the core architecture for LLM-controlled robots that understand both visual scenes and natural language instructions.",
        "chapter": "Vision-Language-Action",
        "section": "VLA Concepts",
        "keywords": ["vla", "vision", "language", "action", "llm", "robot", "multimodal", "control"],
    },
]

# Base URL for images
BASE_IMAGE_URL = "https://enggqasim.github.io/physical-ai-robotics-textbook"


class ImageSearchService:
    """Service for semantic image search using text descriptions."""

    def __init__(self):
        self.embedding_service = get_embedding_service()
        self.image_embeddings: Dict[str, List[float]] = {}
        self._initialize_embeddings()

    def _initialize_embeddings(self):
        """Pre-compute embeddings for all image descriptions."""
        for image in IMAGE_METADATA:
            # Combine title, description, and keywords for rich embedding
            text = f"{image['title']}. {image['description']}. Keywords: {', '.join(image['keywords'])}"
            self.image_embeddings[image['id']] = self.embedding_service.embed_text(text)

    def search_images(
        self,
        query: str,
        limit: int = 3,
        score_threshold: float = 0.5,
    ) -> List[Dict[str, Any]]:
        """Search for relevant images based on query."""
        # Check if query mentions images, diagrams, or visual content
        image_keywords = [
            "diagram", "image", "picture", "visual", "show me", "illustration",
            "figure", "chart", "architecture", "pipeline", "flow"
        ]

        query_lower = query.lower()
        is_image_query = any(kw in query_lower for kw in image_keywords)

        # Get query embedding
        query_embedding = self.embedding_service.embed_text(query)

        # Calculate similarity scores
        results = []
        for image in IMAGE_METADATA:
            image_embedding = self.image_embeddings.get(image['id'])
            if image_embedding:
                score = self._cosine_similarity(query_embedding, image_embedding)

                # Boost score if query explicitly asks for images
                if is_image_query:
                    score *= 1.2

                if score >= score_threshold:
                    results.append({
                        "id": image['id'],
                        "url": f"{BASE_IMAGE_URL}{image['url']}",
                        "title": image['title'],
                        "alt_text": image['alt_text'],
                        "chapter": image['chapter'],
                        "section": image['section'],
                        "score": min(score, 1.0),  # Cap at 1.0
                    })

        # Sort by score and return top results
        results.sort(key=lambda x: x['score'], reverse=True)
        return results[:limit]

    def _cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """Calculate cosine similarity between two vectors."""
        dot_product = sum(a * b for a, b in zip(vec1, vec2))
        norm1 = sum(a * a for a in vec1) ** 0.5
        norm2 = sum(b * b for b in vec2) ** 0.5
        if norm1 == 0 or norm2 == 0:
            return 0.0
        return dot_product / (norm1 * norm2)


# Singleton instance
_image_search_service = None


def get_image_search_service() -> ImageSearchService:
    """Get or create image search service instance."""
    global _image_search_service
    if _image_search_service is None:
        _image_search_service = ImageSearchService()
    return _image_search_service
