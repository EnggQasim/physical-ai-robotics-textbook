"""Orchestrator Agent - Routes requests to specialized agents."""
from typing import Any, Dict, List, Optional
from dataclasses import dataclass
import re

from ..shared.base_agent import BaseAgent, AgentResponse
from ..educational_content import EducationalContentAgent
from ..educational_content.agent import ContentRequest
from ..multimodal_rag import MultimodalRAGAgent
from ..multimodal_rag.agent import ChatRequest
from ..sdd_planning import SDDPlanningAgent
from ..sdd_planning.agent import SDDRequest


@dataclass
class OrchestratorRequest:
    """Request to the orchestrator."""
    message: str
    context: Optional[Dict[str, Any]] = None


class OrchestratorAgent(BaseAgent):
    """Orchestrator that routes requests to specialized agents.

    Routes to:
    - Educational Content Agent: diagrams, GIFs, summaries, mind maps
    - Multimodal RAG Agent: questions, search, knowledge queries
    - SDD Planning Agent: /sp.* commands, specs, plans, tasks

    Routing Strategy:
    1. Command detection: /sp.* commands go to SDD agent
    2. Intent classification: educational vs. question vs. planning
    3. Keyword matching: diagram, gif, summary, mindmap keywords
    """

    def __init__(
        self,
        cache_dir: Optional[str] = None,
        project_dir: Optional[str] = None,
        domain_topics: Optional[List[str]] = None
    ):
        # Initialize specialized agents
        self.educational_agent = EducationalContentAgent(cache_dir=cache_dir)
        self.rag_agent = MultimodalRAGAgent(domain_topics=domain_topics)
        self.sdd_agent = SDDPlanningAgent(project_dir=project_dir or ".")

        super().__init__(
            name="Orchestrator Agent",
            description="Routes requests to specialized agents based on intent",
            system_prompt=self._get_system_prompt()
        )

        # Intent patterns
        self.sdd_patterns = [
            r"^/sp\.(specify|plan|tasks|adr|phr|clarify|analyze|implement)",
            r"create spec", r"write spec", r"generate plan",
            r"create tasks", r"architecture decision"
        ]

        self.educational_patterns = [
            r"diagram", r"flowchart", r"mind\s*map", r"concept\s*map",
            r"gif", r"animation", r"animate", r"visualize",
            r"summary", r"summarize", r"key points"
        ]

        self.rag_patterns = [
            r"what is", r"how does", r"explain", r"tell me",
            r"search", r"find", r"show me", r"where",
            r"why", r"when", r"who"
        ]

    def _get_system_prompt(self) -> str:
        return """You are an Orchestrator Agent that routes requests to specialized agents.

Available Agents:

1. EDUCATIONAL CONTENT AGENT
   Handles: diagrams, flowcharts, mind maps, GIFs, animations, summaries
   Keywords: diagram, visualize, animate, summarize, concept map

2. MULTIMODAL RAG AGENT
   Handles: questions, search, knowledge queries
   Keywords: what, how, explain, search, find, tell me

3. SDD PLANNING AGENT
   Handles: /sp.* commands, specs, plans, tasks, ADRs, PHRs
   Commands: /sp.specify, /sp.plan, /sp.tasks, /sp.adr, /sp.phr

Routing Logic:
1. First check for /sp.* commands -> SDD Agent
2. Check for educational keywords -> Educational Agent
3. Default to RAG Agent for questions

You route requests, aggregate responses, and handle errors.
"""

    def _detect_intent(self, message: str) -> str:
        """Detect the intent of the message."""
        message_lower = message.lower()

        # Check for SDD commands first
        for pattern in self.sdd_patterns:
            if re.search(pattern, message_lower):
                return "sdd"

        # Check for educational content requests
        for pattern in self.educational_patterns:
            if re.search(pattern, message_lower):
                return "educational"

        # Check for RAG patterns
        for pattern in self.rag_patterns:
            if re.search(pattern, message_lower):
                return "rag"

        # Default to RAG for questions
        if "?" in message:
            return "rag"

        return "rag"  # Default

    def _parse_sdd_command(self, message: str) -> Optional[SDDRequest]:
        """Parse an SDD command from message."""
        # Check for /sp.specify
        match = re.search(r"/sp\.specify\s+(.+)", message, re.IGNORECASE)
        if match:
            return SDDRequest(command="specify", description=match.group(1).strip())

        # Check for /sp.plan
        match = re.search(r"/sp\.plan\s+(\S+)", message, re.IGNORECASE)
        if match:
            return SDDRequest(command="plan", feature=match.group(1).strip())

        # Check for /sp.tasks
        match = re.search(r"/sp\.tasks\s+(\S+)", message, re.IGNORECASE)
        if match:
            return SDDRequest(command="tasks", feature=match.group(1).strip())

        # Check for /sp.adr
        match = re.search(r"/sp\.adr\s+(.+)", message, re.IGNORECASE)
        if match:
            parts = match.group(1).strip().split(" - ", 1)
            return SDDRequest(
                command="adr",
                title=parts[0],
                context=parts[1] if len(parts) > 1 else "Context needed"
            )

        return None

    def _parse_educational_request(self, message: str) -> Optional[ContentRequest]:
        """Parse an educational content request."""
        message_lower = message.lower()

        # Detect content type
        if any(kw in message_lower for kw in ["diagram", "flowchart", "concept map", "visualize"]):
            # Extract concept (text after keywords)
            concept = re.sub(r"(create|generate|make|draw|show)\s+(a\s+)?(diagram|flowchart|concept map|visualization)\s+(for|of|about)?\s*", "", message, flags=re.IGNORECASE)
            concept = concept.strip() or "General concept"

            # Detect diagram type
            diagram_type = "concept_map"
            if "flowchart" in message_lower:
                diagram_type = "flowchart"
            elif "mind map" in message_lower:
                diagram_type = "mind_map"
            elif "architecture" in message_lower:
                diagram_type = "architecture"
            elif "sequence" in message_lower:
                diagram_type = "sequence"

            return ContentRequest(
                content_type="diagram",
                concept=concept,
                diagram_type=diagram_type
            )

        elif any(kw in message_lower for kw in ["gif", "animate", "animation"]):
            # Parse steps if provided
            concept = re.sub(r"(create|generate|make)\s+(a\s+)?(gif|animation)\s+(for|of|about|showing)?\s*", "", message, flags=re.IGNORECASE)
            return ContentRequest(
                content_type="gif",
                concept=concept.strip() or "Process",
                steps=None  # Would need to extract steps
            )

        elif any(kw in message_lower for kw in ["summary", "summarize", "key points"]):
            return ContentRequest(
                content_type="summary",
                content=message
            )

        elif "mind map" in message_lower:
            return ContentRequest(
                content_type="mindmap",
                content=message
            )

        return None

    async def route(self, message: str, context: Optional[Dict[str, Any]] = None) -> AgentResponse:
        """Route a message to the appropriate agent."""
        intent = self._detect_intent(message)

        try:
            if intent == "sdd":
                request = self._parse_sdd_command(message)
                if request:
                    return await self.sdd_agent.process(request)
                else:
                    return AgentResponse(
                        success=False,
                        error="Could not parse SDD command. Use /sp.specify, /sp.plan, /sp.tasks, or /sp.adr"
                    )

            elif intent == "educational":
                request = self._parse_educational_request(message)
                if request:
                    return await self.educational_agent.process(request)
                else:
                    return AgentResponse(
                        success=False,
                        error="Could not parse educational content request"
                    )

            else:  # RAG
                session_id = context.get("session_id") if context else None
                request = ChatRequest(
                    message=message,
                    session_id=session_id
                )
                return await self.rag_agent.process(request)

        except Exception as e:
            return AgentResponse(success=False, error=str(e))

    async def process(self, request: OrchestratorRequest) -> AgentResponse:
        """Process an orchestrator request."""
        return await self.route(request.message, request.context)

    def get_capabilities(self) -> List[str]:
        """Return combined capabilities of all agents."""
        capabilities = ["route_requests", "multi_agent_orchestration"]
        capabilities.extend([f"educational:{c}" for c in self.educational_agent.get_capabilities()])
        capabilities.extend([f"rag:{c}" for c in self.rag_agent.get_capabilities()])
        capabilities.extend([f"sdd:{c}" for c in self.sdd_agent.get_capabilities()])
        return capabilities

    def get_agents(self) -> Dict[str, BaseAgent]:
        """Get all specialized agents."""
        return {
            "educational": self.educational_agent,
            "rag": self.rag_agent,
            "sdd": self.sdd_agent
        }
