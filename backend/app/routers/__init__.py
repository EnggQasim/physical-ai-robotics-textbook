"""Routers package."""
from .chat import router as chat_router
from .health import router as health_router
from .diagram import router as diagram_router

__all__ = ["chat_router", "health_router", "diagram_router"]
