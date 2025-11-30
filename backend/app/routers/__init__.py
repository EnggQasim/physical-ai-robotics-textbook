"""Routers package."""
from .chat import router as chat_router
from .health import router as health_router
from .diagram import router as diagram_router
from .podcast import router as podcast_router
from .auth import router as auth_router

__all__ = ["chat_router", "health_router", "diagram_router", "podcast_router", "auth_router"]
