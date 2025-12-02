"""Tools for SDD Planning Agent."""
import os
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field

from openai import AsyncOpenAI

from ..shared.config import get_settings


def slugify(text: str) -> str:
    """Convert text to slug format."""
    text = text.lower()
    text = re.sub(r'[^a-z0-9\s-]', '', text)
    text = re.sub(r'[\s_]+', '-', text)
    return text.strip('-')


@dataclass
class SpecResult:
    """Result from spec generation."""
    success: bool
    filepath: Optional[str] = None
    content: Optional[str] = None
    error: Optional[str] = None


@dataclass
class PlanResult:
    """Result from plan generation."""
    success: bool
    filepath: Optional[str] = None
    content: Optional[str] = None
    decisions: List[Dict[str, Any]] = field(default_factory=list)
    error: Optional[str] = None


@dataclass
class TaskResult:
    """Result from task generation."""
    success: bool
    filepath: Optional[str] = None
    content: Optional[str] = None
    task_count: int = 0
    error: Optional[str] = None


@dataclass
class ADRResult:
    """Result from ADR creation."""
    success: bool
    filepath: Optional[str] = None
    adr_id: Optional[int] = None
    error: Optional[str] = None


@dataclass
class PHRResult:
    """Result from PHR creation."""
    success: bool
    filepath: Optional[str] = None
    phr_id: Optional[int] = None
    error: Optional[str] = None


class SpecWriter:
    """Generate feature specifications."""

    SPEC_TEMPLATE = '''# Feature Specification: {title}

**Feature Branch**: `{branch}`
**Created**: {date}
**Status**: Draft
**Input**: User description: "{description}"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - {story_title} (Priority: P1)

{story_description}

**Why this priority**: {priority_rationale}

**Independent Test**: {test_description}

**Acceptance Scenarios**:

{acceptance_scenarios}

---

### Edge Cases

{edge_cases}

## Requirements *(mandatory)*

### Functional Requirements

{functional_requirements}

### Non-Functional Requirements

{non_functional_requirements}

### Key Entities

{entities}

## Assumptions

{assumptions}

## Success Criteria *(mandatory)*

### Measurable Outcomes

{success_criteria}
'''

    def __init__(self, specs_dir: str = "specs"):
        settings = get_settings()
        self.specs_dir = Path(specs_dir)
        self.specs_dir.mkdir(parents=True, exist_ok=True)

        if settings.openai_api_key:
            self.client = AsyncOpenAI(api_key=settings.openai_api_key)
            self.model = settings.openai_chat_model
        else:
            self.client = None

    async def generate(
        self,
        description: str,
        feature_name: Optional[str] = None
    ) -> SpecResult:
        """Generate a feature specification from description."""
        if not self.client:
            return SpecResult(success=False, error="OpenAI API not configured")

        # Generate feature name if not provided
        if not feature_name:
            feature_name = await self._generate_feature_name(description)

        branch = slugify(feature_name)
        feature_dir = self.specs_dir / branch
        feature_dir.mkdir(parents=True, exist_ok=True)

        prompt = f"""Analyze this feature description and generate a structured specification.

Description: {description}

Generate JSON with:
{{
    "title": "Feature title",
    "story_title": "Main user story title",
    "story_description": "User story description",
    "priority_rationale": "Why P1 priority",
    "test_description": "How to test independently",
    "acceptance_scenarios": ["Given... When... Then...", ...],
    "edge_cases": ["Case 1: handling", ...],
    "functional_requirements": ["FR-001: System MUST...", ...],
    "non_functional_requirements": ["NFR-001: Performance...", ...],
    "entities": ["Entity: description", ...],
    "assumptions": ["Assumption 1", ...],
    "success_criteria": ["SC-001: Metric target (measurement)", ...]
}}
"""

        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"}
            )

            import json
            data = json.loads(response.choices[0].message.content)

            # Format the spec
            content = self.SPEC_TEMPLATE.format(
                title=data.get("title", feature_name),
                branch=branch,
                date=datetime.now().strftime("%Y-%m-%d"),
                description=description,
                story_title=data.get("story_title", "Primary User Story"),
                story_description=data.get("story_description", ""),
                priority_rationale=data.get("priority_rationale", ""),
                test_description=data.get("test_description", ""),
                acceptance_scenarios="\n".join([
                    f"{i+1}. **Given** {s.replace('Given ', '').replace('When ', ', **When** ').replace('Then ', ', **Then** ')}"
                    for i, s in enumerate(data.get("acceptance_scenarios", []))
                ]),
                edge_cases="\n".join([f"- {c}" for c in data.get("edge_cases", [])]),
                functional_requirements="\n".join([f"- **{r.split(':')[0]}**: {':'.join(r.split(':')[1:]).strip()}" for r in data.get("functional_requirements", [])]),
                non_functional_requirements="\n".join([f"- **{r.split(':')[0]}**: {':'.join(r.split(':')[1:]).strip()}" for r in data.get("non_functional_requirements", [])]),
                entities="\n".join([f"- **{e.split(':')[0]}**: {':'.join(e.split(':')[1:]).strip()}" for e in data.get("entities", [])]),
                assumptions="\n".join([f"- {a}" for a in data.get("assumptions", [])]),
                success_criteria="\n".join([f"- **{c.split(':')[0]}**: {':'.join(c.split(':')[1:]).strip()}" for c in data.get("success_criteria", [])])
            )

            filepath = feature_dir / "spec.md"
            with open(filepath, "w") as f:
                f.write(content)

            return SpecResult(
                success=True,
                filepath=str(filepath),
                content=content
            )

        except Exception as e:
            return SpecResult(success=False, error=str(e))

    async def _generate_feature_name(self, description: str) -> str:
        """Generate a feature name from description."""
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[{
                    "role": "user",
                    "content": f"Generate a short (2-4 words) feature name for: {description}\nRespond with just the name, no quotes."
                }]
            )
            return response.choices[0].message.content.strip()
        except:
            return "new-feature"


class PlanGenerator:
    """Generate implementation plans."""

    PLAN_TEMPLATE = '''# Implementation Plan: {title}

**Feature**: `{branch}`
**Spec**: specs/{branch}/spec.md
**Created**: {date}

## Architecture Decisions

{decisions}

## Technical Design

### Components

{components}

### Interfaces

{interfaces}

### Data Flow

{data_flow}

## NFR Budgets

| Requirement | Budget |
|-------------|--------|
{nfr_table}

## Risks

| Risk | Mitigation |
|------|------------|
{risks_table}

## Implementation Approach

{implementation_approach}
'''

    def __init__(self, specs_dir: str = "specs"):
        settings = get_settings()
        self.specs_dir = Path(specs_dir)

        if settings.openai_api_key:
            self.client = AsyncOpenAI(api_key=settings.openai_api_key)
            self.model = settings.openai_chat_model
        else:
            self.client = None

    async def generate(
        self,
        feature_branch: str,
        spec_content: Optional[str] = None
    ) -> PlanResult:
        """Generate an implementation plan from a spec."""
        if not self.client:
            return PlanResult(success=False, error="OpenAI API not configured")

        # Read spec if not provided
        if not spec_content:
            spec_path = self.specs_dir / feature_branch / "spec.md"
            if spec_path.exists():
                with open(spec_path, "r") as f:
                    spec_content = f.read()
            else:
                return PlanResult(success=False, error=f"Spec not found: {spec_path}")

        prompt = f"""Analyze this feature specification and generate an implementation plan.

Specification:
{spec_content}

Generate JSON with:
{{
    "title": "Plan title",
    "decisions": [
        {{
            "title": "Decision title",
            "context": "Why needed",
            "options": [{{"name": "Option A", "pros": [...], "cons": [...]}}],
            "chosen": "Option A",
            "rationale": "Why chosen"
        }}
    ],
    "components": ["Component: responsibility", ...],
    "interfaces": ["POST /api/endpoint - description", ...],
    "data_flow": "Description of data flow",
    "nfr_budgets": [{{"requirement": "Response time", "budget": "< 3s"}}],
    "risks": [{{"risk": "Risk description", "mitigation": "Mitigation"}}],
    "implementation_approach": "Step by step approach"
}}
"""

        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"}
            )

            import json
            data = json.loads(response.choices[0].message.content)

            # Format decisions
            decisions_text = ""
            for d in data.get("decisions", []):
                decisions_text += f"### Decision: {d['title']}\n\n"
                decisions_text += f"**Context**: {d['context']}\n\n"
                decisions_text += "**Options Considered**:\n\n"
                decisions_text += "| Option | Pros | Cons |\n|--------|------|------|\n"
                for opt in d.get("options", []):
                    pros = ", ".join(opt.get("pros", []))
                    cons = ", ".join(opt.get("cons", []))
                    decisions_text += f"| {opt['name']} | {pros} | {cons} |\n"
                decisions_text += f"\n**Decision**: {d.get('chosen', 'TBD')}\n\n"
                decisions_text += f"**Rationale**: {d.get('rationale', '')}\n\n---\n\n"

            # Format plan
            content = self.PLAN_TEMPLATE.format(
                title=data.get("title", feature_branch),
                branch=feature_branch,
                date=datetime.now().strftime("%Y-%m-%d"),
                decisions=decisions_text,
                components="\n".join([f"- {c}" for c in data.get("components", [])]),
                interfaces="\n".join([f"```\n{i}\n```" for i in data.get("interfaces", [])]),
                data_flow=data.get("data_flow", ""),
                nfr_table="\n".join([f"| {n['requirement']} | {n['budget']} |" for n in data.get("nfr_budgets", [])]),
                risks_table="\n".join([f"| {r['risk']} | {r['mitigation']} |" for r in data.get("risks", [])]),
                implementation_approach=data.get("implementation_approach", "")
            )

            feature_dir = self.specs_dir / feature_branch
            feature_dir.mkdir(parents=True, exist_ok=True)
            filepath = feature_dir / "plan.md"

            with open(filepath, "w") as f:
                f.write(content)

            return PlanResult(
                success=True,
                filepath=str(filepath),
                content=content,
                decisions=data.get("decisions", [])
            )

        except Exception as e:
            return PlanResult(success=False, error=str(e))


class TaskGenerator:
    """Generate implementation tasks."""

    TASK_TEMPLATE = '''# Implementation Tasks: {title}

**Feature**: `{branch}`
**Plan**: specs/{branch}/plan.md
**Generated**: {date}

## Task List

{tasks}

## Summary

- **Total Tasks**: {task_count}
- **P1 Tasks**: {p1_count}
- **P2 Tasks**: {p2_count}
- **P3 Tasks**: {p3_count}
'''

    def __init__(self, specs_dir: str = "specs"):
        settings = get_settings()
        self.specs_dir = Path(specs_dir)

        if settings.openai_api_key:
            self.client = AsyncOpenAI(api_key=settings.openai_api_key)
            self.model = settings.openai_chat_model
        else:
            self.client = None

    async def generate(
        self,
        feature_branch: str,
        spec_content: Optional[str] = None,
        plan_content: Optional[str] = None
    ) -> TaskResult:
        """Generate tasks from spec and plan."""
        if not self.client:
            return TaskResult(success=False, error="OpenAI API not configured")

        feature_dir = self.specs_dir / feature_branch

        # Read spec if not provided
        if not spec_content:
            spec_path = feature_dir / "spec.md"
            if spec_path.exists():
                with open(spec_path, "r") as f:
                    spec_content = f.read()

        # Read plan if not provided
        if not plan_content:
            plan_path = feature_dir / "plan.md"
            if plan_path.exists():
                with open(plan_path, "r") as f:
                    plan_content = f.read()

        prompt = f"""Analyze the specification and plan, then generate implementation tasks.

Specification:
{spec_content or 'Not available'}

Plan:
{plan_content or 'Not available'}

Generate JSON with:
{{
    "title": "Tasks title",
    "tasks": [
        {{
            "id": 1,
            "title": "Task title",
            "priority": "P1",
            "depends_on": [],
            "description": "What to implement",
            "files": ["path/to/file.ts: changes"],
            "test_cases": [
                {{"description": "Test", "input": "...", "expected": "..."}}
            ],
            "acceptance_criteria": ["Criterion 1", ...]
        }}
    ]
}}

Order tasks by dependencies. Include 5-15 tasks.
"""

        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"}
            )

            import json
            data = json.loads(response.choices[0].message.content)

            # Format tasks
            tasks_text = ""
            p1_count = p2_count = p3_count = 0

            for task in data.get("tasks", []):
                priority = task.get("priority", "P2")
                if priority == "P1":
                    p1_count += 1
                elif priority == "P2":
                    p2_count += 1
                else:
                    p3_count += 1

                depends = ", ".join([f"Task {d}" for d in task.get("depends_on", [])]) or "None"

                tasks_text += f"### Task {task['id']}: {task['title']}\n"
                tasks_text += f"**Priority**: {priority}\n"
                tasks_text += f"**Depends On**: {depends}\n"
                tasks_text += f"**Status**: [ ] Pending\n\n"
                tasks_text += f"**Description**: {task.get('description', '')}\n\n"

                if task.get("files"):
                    tasks_text += "**Files to Modify**:\n"
                    for f in task["files"]:
                        tasks_text += f"- `{f}`\n"
                    tasks_text += "\n"

                if task.get("test_cases"):
                    tasks_text += "**Test Cases**:\n"
                    for tc in task["test_cases"]:
                        tasks_text += f"- [ ] Test: {tc['description']}\n"
                        tasks_text += f"  - Input: {tc.get('input', 'N/A')}\n"
                        tasks_text += f"  - Expected: {tc.get('expected', 'N/A')}\n"
                    tasks_text += "\n"

                if task.get("acceptance_criteria"):
                    tasks_text += "**Acceptance Criteria**:\n"
                    for ac in task["acceptance_criteria"]:
                        tasks_text += f"- [ ] {ac}\n"

                tasks_text += "\n---\n\n"

            task_count = len(data.get("tasks", []))

            content = self.TASK_TEMPLATE.format(
                title=data.get("title", feature_branch),
                branch=feature_branch,
                date=datetime.now().strftime("%Y-%m-%d"),
                tasks=tasks_text,
                task_count=task_count,
                p1_count=p1_count,
                p2_count=p2_count,
                p3_count=p3_count
            )

            feature_dir.mkdir(parents=True, exist_ok=True)
            filepath = feature_dir / "tasks.md"

            with open(filepath, "w") as f:
                f.write(content)

            return TaskResult(
                success=True,
                filepath=str(filepath),
                content=content,
                task_count=task_count
            )

        except Exception as e:
            return TaskResult(success=False, error=str(e))


class ADRManager:
    """Manage Architecture Decision Records."""

    ADR_TEMPLATE = '''# ADR-{id:03d}: {title}

> **Scope**: {scope}

- **Status:** {status}
- **Date:** {date}
- **Feature:** {feature}
- **Context:** {context}

## Decision

{decision}

## Consequences

### Positive

{positive}

### Negative

{negative}

## Alternatives Considered

{alternatives}

## References

- Feature Spec: `specs/{feature}/spec.md`
- Implementation Plan: `specs/{feature}/plan.md`
'''

    def __init__(self, adr_dir: str = "history/adr"):
        self.adr_dir = Path(adr_dir)
        self.adr_dir.mkdir(parents=True, exist_ok=True)

    def _get_next_id(self) -> int:
        """Get next ADR ID."""
        existing = list(self.adr_dir.glob("*.md"))
        if not existing:
            return 1

        ids = []
        for f in existing:
            match = re.match(r"(\d+)-", f.name)
            if match:
                ids.append(int(match.group(1)))

        return max(ids, default=0) + 1

    async def create(
        self,
        title: str,
        context: str,
        decision: str,
        feature: Optional[str] = None,
        positive: Optional[List[str]] = None,
        negative: Optional[List[str]] = None,
        alternatives: Optional[List[Dict[str, str]]] = None
    ) -> ADRResult:
        """Create a new ADR."""
        try:
            adr_id = self._get_next_id()
            slug = slugify(title)

            content = self.ADR_TEMPLATE.format(
                id=adr_id,
                title=title,
                scope=title,
                status="Accepted",
                date=datetime.now().strftime("%Y-%m-%d"),
                feature=feature or "general",
                context=context,
                decision=decision,
                positive="\n".join([f"- {p}" for p in (positive or ["TBD"])]),
                negative="\n".join([f"- {n}" for n in (negative or ["TBD"])]),
                alternatives="\n".join([
                    f"### {a.get('name', 'Alternative')}\n- **Pros**: {a.get('pros', 'N/A')}\n- **Cons**: {a.get('cons', 'N/A')}\n- **Why rejected**: {a.get('rejected', 'N/A')}"
                    for a in (alternatives or [])
                ]) or "None documented."
            )

            filename = f"{adr_id:03d}-{slug}.md"
            filepath = self.adr_dir / filename

            with open(filepath, "w") as f:
                f.write(content)

            return ADRResult(
                success=True,
                filepath=str(filepath),
                adr_id=adr_id
            )

        except Exception as e:
            return ADRResult(success=False, error=str(e))

    def should_suggest_adr(self, decision_context: str) -> bool:
        """Check if an ADR should be suggested."""
        impact_keywords = [
            "framework", "architecture", "database", "schema",
            "authentication", "authorization", "api", "protocol",
            "deployment", "infrastructure", "security", "platform",
            "stack", "technology", "library"
        ]

        context_lower = decision_context.lower()

        has_impact = any(kw in context_lower for kw in impact_keywords)
        has_alternatives = "option" in context_lower or "alternative" in context_lower or "vs" in context_lower
        is_cross_cutting = "system" in context_lower or "design" in context_lower or "architecture" in context_lower

        return has_impact and (has_alternatives or is_cross_cutting)


class PHRManager:
    """Manage Prompt History Records."""

    PHR_TEMPLATE = '''---
id: {id}
title: "{title}"
stage: {stage}
date: {date}
surface: agent
model: {model}
feature: {feature}
branch: {branch}
user: system
labels: {labels}
links:
  spec: {spec_link}
  adr: {adr_link}
files: {files}
tests: {tests}
---

# {title}

## Prompt

```
{prompt_text}
```

## Response

{response_text}

## Outcome

{outcome}
'''

    STAGES = [
        "constitution", "spec", "plan", "tasks",
        "red", "green", "refactor", "explainer", "misc", "general"
    ]

    def __init__(self, phr_dir: str = "history/prompts"):
        self.phr_dir = Path(phr_dir)
        self.phr_dir.mkdir(parents=True, exist_ok=True)

    def _get_route(self, stage: str, feature: Optional[str]) -> Path:
        """Get the route directory for a PHR."""
        if stage == "constitution":
            route = self.phr_dir / "constitution"
        elif feature and stage in ["spec", "plan", "tasks", "red", "green", "refactor", "explainer", "misc"]:
            route = self.phr_dir / feature
        else:
            route = self.phr_dir / "general"

        route.mkdir(parents=True, exist_ok=True)
        return route

    def _get_next_id(self, route: Path) -> int:
        """Get next PHR ID in route."""
        existing = list(route.glob("*.prompt.md"))
        if not existing:
            return 1

        ids = []
        for f in existing:
            match = re.match(r"(\d+)-", f.name)
            if match:
                ids.append(int(match.group(1)))

        return max(ids, default=0) + 1

    async def create(
        self,
        title: str,
        stage: str,
        prompt_text: str,
        response_text: str,
        feature: Optional[str] = None,
        branch: Optional[str] = None,
        labels: Optional[List[str]] = None,
        files: Optional[List[str]] = None,
        tests: Optional[List[str]] = None,
        outcome: Optional[str] = None
    ) -> PHRResult:
        """Create a new PHR."""
        if stage not in self.STAGES:
            return PHRResult(success=False, error=f"Invalid stage: {stage}")

        try:
            route = self._get_route(stage, feature)
            phr_id = self._get_next_id(route)
            slug = slugify(title)

            settings = get_settings()

            content = self.PHR_TEMPLATE.format(
                id=phr_id,
                title=title,
                stage=stage,
                date=datetime.now().strftime("%Y-%m-%d"),
                model=settings.openai_chat_model,
                feature=feature or "none",
                branch=branch or "main",
                labels=str(labels or []),
                spec_link=f"specs/{feature}/spec.md" if feature else "null",
                adr_link="null",
                files="\n".join([f"  - {f}" for f in (files or [])]) or "  - none",
                tests="\n".join([f"  - {t}" for t in (tests or [])]) or "  - none",
                prompt_text=prompt_text,
                response_text=response_text,
                outcome=outcome or "Completed successfully."
            )

            filename = f"{phr_id:04d}-{slug}.{stage}.prompt.md"
            filepath = route / filename

            with open(filepath, "w") as f:
                f.write(content)

            return PHRResult(
                success=True,
                filepath=str(filepath),
                phr_id=phr_id
            )

        except Exception as e:
            return PHRResult(success=False, error=str(e))
