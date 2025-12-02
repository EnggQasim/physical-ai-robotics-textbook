"""Base agent class and common utilities."""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional
from datetime import datetime
import json
import hashlib


@dataclass
class AgentTool:
    """Definition of a tool available to an agent."""
    name: str
    description: str
    parameters: Dict[str, Any]
    execute: Callable
    required_params: List[str] = field(default_factory=list)

    def to_openai_format(self) -> Dict[str, Any]:
        """Convert to OpenAI function calling format."""
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": {
                    "type": "object",
                    "properties": self.parameters,
                    "required": self.required_params
                }
            }
        }

    def to_anthropic_format(self) -> Dict[str, Any]:
        """Convert to Anthropic tool use format."""
        return {
            "name": self.name,
            "description": self.description,
            "input_schema": {
                "type": "object",
                "properties": self.parameters,
                "required": self.required_params
            }
        }


@dataclass
class AgentResponse:
    """Standard response from an agent."""
    success: bool
    data: Any = None
    error: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "success": self.success,
            "data": self.data,
            "error": self.error,
            "metadata": self.metadata,
            "timestamp": self.timestamp
        }

    def to_json(self) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict(), default=str)


class BaseAgent(ABC):
    """Base class for all agents."""

    def __init__(
        self,
        name: str,
        description: str,
        tools: Optional[List[AgentTool]] = None,
        system_prompt: Optional[str] = None
    ):
        self.name = name
        self.description = description
        self.tools = tools or []
        self.system_prompt = system_prompt or self._default_system_prompt()
        self._cache: Dict[str, Any] = {}

    def _default_system_prompt(self) -> str:
        """Default system prompt for the agent."""
        return f"""You are {self.name}, a specialized AI agent.

Description: {self.description}

Available tools:
{self._format_tools()}

Guidelines:
- Use tools when appropriate to complete tasks
- Provide clear, accurate responses
- Cite sources when available
- Ask for clarification if needed
"""

    def _format_tools(self) -> str:
        """Format tools for system prompt."""
        if not self.tools:
            return "No tools available."
        return "\n".join([
            f"- {tool.name}: {tool.description}"
            for tool in self.tools
        ])

    def add_tool(self, tool: AgentTool) -> None:
        """Add a tool to the agent."""
        self.tools.append(tool)

    def get_tool(self, name: str) -> Optional[AgentTool]:
        """Get a tool by name."""
        for tool in self.tools:
            if tool.name == name:
                return tool
        return None

    async def execute_tool(self, name: str, **kwargs) -> AgentResponse:
        """Execute a tool by name."""
        tool = self.get_tool(name)
        if not tool:
            return AgentResponse(
                success=False,
                error=f"Tool '{name}' not found"
            )
        try:
            result = await tool.execute(**kwargs)
            return AgentResponse(success=True, data=result)
        except Exception as e:
            return AgentResponse(success=False, error=str(e))

    def _get_cache_key(self, *args) -> str:
        """Generate a cache key from arguments."""
        content = json.dumps(args, sort_keys=True, default=str)
        return hashlib.md5(content.encode()).hexdigest()[:12]

    def _get_cached(self, key: str) -> Optional[Any]:
        """Get cached value."""
        return self._cache.get(key)

    def _set_cached(self, key: str, value: Any) -> None:
        """Set cached value."""
        self._cache[key] = value

    @abstractmethod
    async def process(self, input_data: Any) -> AgentResponse:
        """Process input and return response. Must be implemented by subclasses."""
        pass

    @abstractmethod
    def get_capabilities(self) -> List[str]:
        """Return list of agent capabilities."""
        pass
