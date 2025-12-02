"""Educational Content Generator Agent.

Generates pedagogically-sound educational materials:
- Diagrams (concept maps, flowcharts, mind maps, etc.)
- Animated GIFs (step-by-step workflows)
- Summaries (structured text summaries)
- Mind Maps (interactive concept visualizations)
"""
from .agent import EducationalContentAgent
from .tools import (
    DiagramGenerator,
    GifGenerator,
    SummaryGenerator,
    MindMapGenerator,
)

__all__ = [
    "EducationalContentAgent",
    "DiagramGenerator",
    "GifGenerator",
    "SummaryGenerator",
    "MindMapGenerator",
]
