"""Shared configuration for all agents."""
import os
from typing import Optional
from pydantic_settings import BaseSettings
from functools import lru_cache


class Settings(BaseSettings):
    """Agent configuration settings."""

    # API Keys
    openai_api_key: Optional[str] = None
    anthropic_api_key: Optional[str] = None
    gemini_api_key: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_url: Optional[str] = None

    # Model Configuration
    openai_embedding_model: str = "text-embedding-3-small"
    openai_chat_model: str = "gpt-4o-mini"
    anthropic_model: str = "claude-sonnet-4-20250514"
    gemini_model: str = "gemini-2.0-flash-exp"

    # Vector DB Configuration
    qdrant_collection: str = "knowledge_base"
    text_embedding_size: int = 1536
    image_embedding_size: int = 512

    # Domain Configuration
    domain_name: str = "general"
    domain_topics: list = []
    primary_color: str = "#76b900"

    # Cache Configuration
    cache_dir: str = "./cache"
    enable_caching: bool = True

    # Rate Limiting
    max_requests_per_minute: int = 60

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()


# Design Guidelines (Research-Based)
DESIGN_GUIDELINES = {
    "visual_hierarchy": "Animate/highlight primary elements first, supporting details second",
    "color_scheme": {
        "primary": "#76b900",      # Green - main concepts, success
        "secondary": "#4285f4",    # Blue - processes, actions
        "accent": "#ff9800",       # Orange - highlights, warnings
        "success": "#4caf50",      # Green - completed states
        "info": "#2196f3",         # Light blue - informational
        "neutral": "#757575"       # Gray - supporting elements
    },
    "animation_timing": {
        "step_duration_ms": 2500,
        "transition_ms": 300,
        "max_steps": 8
    },
    "accessibility": {
        "min_contrast_ratio": 4.5,
        "font_size_min": 14,
        "clear_labels": True
    }
}

# Diagram Types
DIAGRAM_TYPES = {
    "concept_map": {
        "description": "Shows relationships between interconnected concepts",
        "best_for": ["understanding relationships", "big picture view", "brainstorming"],
        "style": "nodes connected by labeled arrows showing relationships"
    },
    "flowchart": {
        "description": "Sequential process or workflow with decision points",
        "best_for": ["processes", "algorithms", "decision making", "troubleshooting"],
        "style": "rectangles for actions, diamonds for decisions, arrows for flow"
    },
    "mind_map": {
        "description": "Hierarchical branching from a central concept",
        "best_for": ["topic overview", "note-taking", "organizing ideas"],
        "style": "central node with radiating branches, color-coded categories"
    },
    "architecture": {
        "description": "Layered or component-based system structure",
        "best_for": ["system design", "software architecture", "hardware layout"],
        "style": "stacked layers or interconnected boxes with clear boundaries"
    },
    "sequence": {
        "description": "Time-ordered interactions between components",
        "best_for": ["communication protocols", "API calls", "message passing"],
        "style": "vertical lifelines with horizontal arrows showing messages"
    },
    "comparison": {
        "description": "Side-by-side comparison of concepts or features",
        "best_for": ["choosing between options", "understanding differences"],
        "style": "parallel columns or Venn diagram with clear labels"
    },
    "tree": {
        "description": "Hierarchical parent-child relationships",
        "best_for": ["classification", "organizational structure", "decision trees"],
        "style": "root at top, branches descending with clear hierarchy"
    },
    "cycle": {
        "description": "Circular/iterative process with repeating steps",
        "best_for": ["feedback loops", "life cycles", "iterative processes"],
        "style": "circular arrangement with arrows showing continuous flow"
    }
}
