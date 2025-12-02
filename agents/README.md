# Reusable Intelligence Agents

Production-ready AI agents extracted from the Physical AI Robotics Textbook project.

## Installation

```bash
cd agents
pip install -r requirements.txt
```

## Environment Setup

Create a `.env` file:

```env
OPENAI_API_KEY=your-openai-key
GEMINI_API_KEY=your-gemini-key
QDRANT_URL=your-qdrant-url
QDRANT_API_KEY=your-qdrant-key
```

## Available Agents

### 1. Educational Content Agent

Generates pedagogically-sound educational materials.

```python
from agents import EducationalContentAgent
from agents.educational_content.agent import ContentRequest

agent = EducationalContentAgent()

# Generate a diagram
request = ContentRequest(
    content_type="diagram",
    concept="ROS2 Publisher-Subscriber Pattern",
    diagram_type="flowchart"
)
result = await agent.process(request)
print(result.data)

# Generate an animated GIF
request = ContentRequest(
    content_type="gif",
    concept="SLAM Process",
    steps=[
        "Sensor captures environment",
        "Extract features",
        "Match with known map",
        "Update robot position",
        "Add new features to map"
    ]
)
result = await agent.process(request)

# Generate a summary
request = ContentRequest(
    content_type="summary",
    content="Your long text content here..."
)
result = await agent.process(request)

# Generate a mind map
request = ContentRequest(
    content_type="mindmap",
    content="Your content here...",
    concept="Central Topic"
)
result = await agent.process(request)
```

**Diagram Types:**
- `concept_map` - Shows relationships between concepts
- `flowchart` - Sequential processes
- `mind_map` - Hierarchical branching
- `architecture` - System components
- `sequence` - Time-ordered interactions
- `comparison` - Side-by-side features
- `tree` - Parent-child hierarchies
- `cycle` - Circular processes

### 2. Multimodal RAG Agent

Retrieval-Augmented Generation with text and image search.

```python
from agents import MultimodalRAGAgent
from agents.multimodal_rag.agent import ChatRequest

agent = MultimodalRAGAgent(domain_topics=["ROS2", "Robotics", "AI"])

# Chat with citations
request = ChatRequest(
    message="What is the difference between ROS topics and services?",
    session_id=None  # Creates new session
)
result = await agent.process(request)
print(result.data["response"])
print(result.data["sources"])

# Search for text
results = await agent._search_text("URDF robot description")

# Search for images
results = await agent._search_images("Show me the SLAM diagram")

# Index content
await agent._index_content(
    content="Your content to index...",
    content_type="text",
    metadata={"chapter_id": "module-1", "section": "intro"}
)
```

### 3. SDD Planning Agent

Spec-Driven Development workflow automation.

```python
from agents import SDDPlanningAgent
from agents.sdd_planning.agent import SDDRequest

agent = SDDPlanningAgent(project_dir="./my-project")

# Create a specification
request = SDDRequest(
    command="specify",
    description="Add user authentication with OAuth2 support"
)
result = await agent.process(request)
print(f"Spec created: {result.data['filepath']}")

# Generate implementation plan
request = SDDRequest(
    command="plan",
    feature="user-authentication"
)
result = await agent.process(request)

# Generate tasks
request = SDDRequest(
    command="tasks",
    feature="user-authentication"
)
result = await agent.process(request)

# Create ADR
request = SDDRequest(
    command="adr",
    title="Authentication Strategy",
    context="Need to decide between JWT and session-based auth",
    content="We will use JWT tokens for stateless authentication"
)
result = await agent.process(request)
```

### 4. Orchestrator Agent

Routes requests to appropriate specialized agents.

```python
from agents import OrchestratorAgent
from agents.orchestrator.agent import OrchestratorRequest

orchestrator = OrchestratorAgent(
    domain_topics=["ROS2", "Robotics"]
)

# Auto-routes to Educational Agent
result = await orchestrator.process(
    OrchestratorRequest(message="Create a flowchart for ROS2 pub/sub")
)

# Auto-routes to RAG Agent
result = await orchestrator.process(
    OrchestratorRequest(message="What is URDF?")
)

# Auto-routes to SDD Agent
result = await orchestrator.process(
    OrchestratorRequest(message="/sp.specify Add dark mode support")
)
```

## CLI Usage

```bash
# Educational Content
python -m agents.cli edu diagram "ROS2 Architecture" --diagram-type architecture
python -m agents.cli edu gif "Build Process" "Step 1; Step 2; Step 3"
python -m agents.cli edu summary "Your content here..."
python -m agents.cli edu types

# RAG
python -m agents.cli rag chat "What is ROS2?"
python -m agents.cli rag search "URDF" --search-type text

# SDD
python -m agents.cli sdd specify "Add user authentication"
python -m agents.cli sdd plan user-authentication
python -m agents.cli sdd tasks user-authentication
python -m agents.cli sdd adr "Auth Strategy" --context "..." --decision "..."

# Orchestrator
python -m agents.cli ask "Create a diagram of the SLAM process"
python -m agents.cli capabilities
```

## Architecture

```
agents/
├── __init__.py              # Package exports
├── cli.py                   # CLI interface
├── requirements.txt         # Dependencies
├── shared/                  # Shared utilities
│   ├── __init__.py
│   ├── config.py            # Configuration & design guidelines
│   └── base_agent.py        # Base agent class
├── educational_content/     # Educational Content Agent
│   ├── __init__.py
│   ├── agent.py             # Main agent
│   └── tools.py             # DiagramGenerator, GifGenerator, etc.
├── multimodal_rag/          # Multimodal RAG Agent
│   ├── __init__.py
│   ├── agent.py             # Main agent
│   └── tools.py             # Embedders, VectorSearch, etc.
├── sdd_planning/            # SDD Planning Agent
│   ├── __init__.py
│   ├── agent.py             # Main agent
│   └── tools.py             # SpecWriter, PlanGenerator, etc.
└── orchestrator/            # Orchestrator Agent
    ├── __init__.py
    └── agent.py             # Routes to specialized agents
```

## Design Guidelines

Based on research from Graphics for Learning, Educational Voice, and cognitive load theory:

```python
DESIGN_GUIDELINES = {
    "color_scheme": {
        "primary": "#76b900",      # Green - main concepts
        "secondary": "#4285f4",    # Blue - processes
        "accent": "#ff9800",       # Orange - highlights
    },
    "animation_timing": {
        "step_duration_ms": 2500,
        "max_steps": 8
    },
    "accessibility": {
        "min_contrast_ratio": 4.5,
        "font_size_min": 14
    }
}
```

## Extending Agents

### Add a Custom Tool

```python
from agents.shared.base_agent import AgentTool

custom_tool = AgentTool(
    name="my_custom_tool",
    description="Does something custom",
    parameters={
        "input": {"type": "string", "description": "Input data"}
    },
    execute=my_custom_function,
    required_params=["input"]
)

agent.add_tool(custom_tool)
```

### Create a Custom Agent

```python
from agents.shared.base_agent import BaseAgent, AgentResponse

class MyCustomAgent(BaseAgent):
    def __init__(self):
        super().__init__(
            name="My Custom Agent",
            description="Does custom things"
        )

    async def process(self, request) -> AgentResponse:
        # Your implementation
        return AgentResponse(success=True, data={"result": "..."})

    def get_capabilities(self) -> list:
        return ["custom_capability"]
```

## License

MIT
