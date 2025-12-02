"""SDD Planning Agent."""
from typing import Any, Dict, List, Optional
from dataclasses import dataclass

from ..shared.base_agent import BaseAgent, AgentResponse, AgentTool
from .tools import (
    SpecWriter,
    PlanGenerator,
    TaskGenerator,
    ADRManager,
    PHRManager,
)


@dataclass
class SDDRequest:
    """Request for SDD workflow."""
    command: str  # specify, plan, tasks, adr, phr
    description: Optional[str] = None
    feature: Optional[str] = None
    title: Optional[str] = None
    context: Optional[str] = None
    content: Optional[str] = None


class SDDPlanningAgent(BaseAgent):
    """Agent specialized in Spec-Driven Development workflows.

    Capabilities:
    - /sp.specify: Create feature specifications
    - /sp.plan: Generate implementation plans
    - /sp.tasks: Create task lists
    - /sp.adr: Manage Architecture Decision Records
    - /sp.phr: Track Prompt History Records

    Workflow: specify -> plan -> tasks -> implement
    """

    def __init__(self, project_dir: str = "."):
        # Initialize tools
        self.spec_writer = SpecWriter(f"{project_dir}/specs")
        self.plan_generator = PlanGenerator(f"{project_dir}/specs")
        self.task_generator = TaskGenerator(f"{project_dir}/specs")
        self.adr_manager = ADRManager(f"{project_dir}/history/adr")
        self.phr_manager = PHRManager(f"{project_dir}/history/prompts")

        # Create tool definitions
        tools = [
            AgentTool(
                name="specify",
                description="Create a feature specification from description",
                parameters={
                    "description": {"type": "string", "description": "Feature description"},
                    "feature_name": {"type": "string", "description": "Optional feature name"}
                },
                execute=self._specify,
                required_params=["description"]
            ),
            AgentTool(
                name="plan",
                description="Generate implementation plan from spec",
                parameters={
                    "feature": {"type": "string", "description": "Feature branch name"},
                    "spec_content": {"type": "string", "description": "Optional spec content"}
                },
                execute=self._plan,
                required_params=["feature"]
            ),
            AgentTool(
                name="tasks",
                description="Generate tasks from spec and plan",
                parameters={
                    "feature": {"type": "string", "description": "Feature branch name"}
                },
                execute=self._tasks,
                required_params=["feature"]
            ),
            AgentTool(
                name="adr",
                description="Create an Architecture Decision Record",
                parameters={
                    "title": {"type": "string", "description": "Decision title"},
                    "context": {"type": "string", "description": "Decision context"},
                    "decision": {"type": "string", "description": "The decision made"},
                    "feature": {"type": "string", "description": "Related feature"}
                },
                execute=self._adr,
                required_params=["title", "context", "decision"]
            ),
            AgentTool(
                name="phr",
                description="Create a Prompt History Record",
                parameters={
                    "title": {"type": "string", "description": "PHR title"},
                    "stage": {"type": "string", "description": "Workflow stage"},
                    "prompt_text": {"type": "string", "description": "The prompt"},
                    "response_text": {"type": "string", "description": "The response"},
                    "feature": {"type": "string", "description": "Related feature"}
                },
                execute=self._phr,
                required_params=["title", "stage", "prompt_text", "response_text"]
            ),
            AgentTool(
                name="check_adr",
                description="Check if a decision should have an ADR",
                parameters={
                    "decision_context": {"type": "string", "description": "Decision context"}
                },
                execute=self._check_adr,
                required_params=["decision_context"]
            )
        ]

        super().__init__(
            name="SDD Planning Agent",
            description="Spec-Driven Development agent for structured software development workflows",
            tools=tools,
            system_prompt=self._get_system_prompt()
        )

    def _get_system_prompt(self) -> str:
        return """You are an SDD Planning Agent specialized in structured software development.

Your workflow commands:
1. /sp.specify - Transform requirements into structured specifications
   - Creates specs/{feature}/spec.md
   - Includes user stories, acceptance criteria, requirements

2. /sp.plan - Generate architecture and implementation plan
   - Creates specs/{feature}/plan.md
   - Documents decisions, components, interfaces, risks

3. /sp.tasks - Create dependency-ordered task list
   - Creates specs/{feature}/tasks.md
   - Includes test cases and acceptance criteria per task

4. /sp.adr - Create Architecture Decision Record
   - Creates history/adr/{id}-{slug}.md
   - Documents context, decision, consequences, alternatives

5. /sp.phr - Create Prompt History Record
   - Creates history/prompts/{route}/{id}-{slug}.prompt.md
   - Records interactions for traceability

Workflow Order: specify -> plan -> tasks -> implement

ADR Detection:
When you detect significant architectural decisions (framework, database,
API design, security approach), suggest creating an ADR.

Three-part test:
1. Impact: Long-term consequences?
2. Alternatives: Multiple viable options?
3. Scope: Cross-cutting system design?

If all true, suggest: "Architectural decision detected: [brief]. Document? Run /sp.adr [title]"

PHR Creation:
After significant interactions, create a PHR to record:
- The prompt/request
- Your response/actions
- Files modified
- Tests added

Quality Standards:
- Every spec must have acceptance scenarios
- Every plan must document risks
- Every task must have test cases
- Every ADR must list alternatives
"""

    async def _specify(
        self,
        description: str,
        feature_name: Optional[str] = None
    ) -> Dict[str, Any]:
        """Create a feature specification."""
        result = await self.spec_writer.generate(
            description=description,
            feature_name=feature_name
        )
        return {
            "success": result.success,
            "filepath": result.filepath,
            "error": result.error
        }

    async def _plan(
        self,
        feature: str,
        spec_content: Optional[str] = None
    ) -> Dict[str, Any]:
        """Generate implementation plan."""
        result = await self.plan_generator.generate(
            feature_branch=feature,
            spec_content=spec_content
        )

        # Check for ADR suggestions
        adr_suggestions = []
        for decision in result.decisions:
            if self.adr_manager.should_suggest_adr(str(decision)):
                adr_suggestions.append(decision.get("title", "Unnamed decision"))

        return {
            "success": result.success,
            "filepath": result.filepath,
            "adr_suggestions": adr_suggestions,
            "error": result.error
        }

    async def _tasks(
        self,
        feature: str
    ) -> Dict[str, Any]:
        """Generate tasks."""
        result = await self.task_generator.generate(feature_branch=feature)
        return {
            "success": result.success,
            "filepath": result.filepath,
            "task_count": result.task_count,
            "error": result.error
        }

    async def _adr(
        self,
        title: str,
        context: str,
        decision: str,
        feature: Optional[str] = None,
        positive: Optional[List[str]] = None,
        negative: Optional[List[str]] = None
    ) -> Dict[str, Any]:
        """Create an ADR."""
        result = await self.adr_manager.create(
            title=title,
            context=context,
            decision=decision,
            feature=feature,
            positive=positive,
            negative=negative
        )
        return {
            "success": result.success,
            "filepath": result.filepath,
            "adr_id": result.adr_id,
            "error": result.error
        }

    async def _phr(
        self,
        title: str,
        stage: str,
        prompt_text: str,
        response_text: str,
        feature: Optional[str] = None,
        files: Optional[List[str]] = None
    ) -> Dict[str, Any]:
        """Create a PHR."""
        result = await self.phr_manager.create(
            title=title,
            stage=stage,
            prompt_text=prompt_text,
            response_text=response_text,
            feature=feature,
            files=files
        )
        return {
            "success": result.success,
            "filepath": result.filepath,
            "phr_id": result.phr_id,
            "error": result.error
        }

    async def _check_adr(
        self,
        decision_context: str
    ) -> Dict[str, Any]:
        """Check if a decision warrants an ADR."""
        should_create = self.adr_manager.should_suggest_adr(decision_context)
        return {
            "should_create_adr": should_create,
            "suggestion": f"Architectural decision detected. Document? Run /sp.adr [decision-title]" if should_create else None
        }

    async def process(self, request: SDDRequest) -> AgentResponse:
        """Process an SDD workflow request."""
        try:
            if request.command == "specify":
                if not request.description:
                    return AgentResponse(success=False, error="Description required for specify")
                result = await self._specify(
                    description=request.description,
                    feature_name=request.feature
                )

            elif request.command == "plan":
                if not request.feature:
                    return AgentResponse(success=False, error="Feature required for plan")
                result = await self._plan(
                    feature=request.feature,
                    spec_content=request.content
                )

            elif request.command == "tasks":
                if not request.feature:
                    return AgentResponse(success=False, error="Feature required for tasks")
                result = await self._tasks(feature=request.feature)

            elif request.command == "adr":
                if not request.title or not request.context:
                    return AgentResponse(success=False, error="Title and context required for ADR")
                result = await self._adr(
                    title=request.title,
                    context=request.context,
                    decision=request.content or "Decision pending",
                    feature=request.feature
                )

            elif request.command == "phr":
                if not request.title or not request.content:
                    return AgentResponse(success=False, error="Title and content required for PHR")
                result = await self._phr(
                    title=request.title,
                    stage=request.context or "general",
                    prompt_text=request.description or "",
                    response_text=request.content,
                    feature=request.feature
                )

            else:
                return AgentResponse(
                    success=False,
                    error=f"Unknown command: {request.command}"
                )

            return AgentResponse(
                success=result.get("success", False),
                data=result,
                error=result.get("error")
            )

        except Exception as e:
            return AgentResponse(success=False, error=str(e))

    def get_capabilities(self) -> List[str]:
        """Return list of agent capabilities."""
        return [
            "specify",
            "plan",
            "tasks",
            "adr",
            "phr",
            "check_adr"
        ]

    def get_workflow_stages(self) -> List[str]:
        """Get the SDD workflow stages."""
        return ["specify", "plan", "tasks", "implement"]
