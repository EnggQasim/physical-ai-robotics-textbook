"""Reusable Intelligence Agents.

A collection of specialized AI agents:
- Educational Content Agent: Diagrams, GIFs, summaries, mind maps
- Multimodal RAG Agent: Text + image search, Q&A with citations
- SDD Planning Agent: Specs, plans, tasks, ADRs, PHRs
- Orchestrator Agent: Routes requests to specialized agents
"""

from .educational_content import EducationalContentAgent
from .multimodal_rag import MultimodalRAGAgent
from .sdd_planning import SDDPlanningAgent
from .orchestrator import OrchestratorAgent
from .shared import get_settings, Settings, BaseAgent, AgentResponse

__version__ = "1.0.0"

__all__ = [
    # Agents
    "EducationalContentAgent",
    "MultimodalRAGAgent",
    "SDDPlanningAgent",
    "OrchestratorAgent",
    # Shared
    "get_settings",
    "Settings",
    "BaseAgent",
    "AgentResponse",
]
