#!/usr/bin/env python3
"""CLI for Reusable Intelligence Agents."""
import asyncio
import json
from typing import Optional

import typer
from rich.console import Console
from rich.panel import Panel
from rich.markdown import Markdown
from rich.table import Table

from . import (
    EducationalContentAgent,
    MultimodalRAGAgent,
    SDDPlanningAgent,
    OrchestratorAgent,
)
from .educational_content.agent import ContentRequest
from .multimodal_rag.agent import ChatRequest
from .sdd_planning.agent import SDDRequest
from .orchestrator.agent import OrchestratorRequest

app = typer.Typer(help="Reusable Intelligence Agents CLI")
console = Console()


# Educational Content Commands
edu_app = typer.Typer(help="Educational Content Agent commands")
app.add_typer(edu_app, name="edu")


@edu_app.command("diagram")
def generate_diagram(
    concept: str = typer.Argument(..., help="Concept to visualize"),
    diagram_type: str = typer.Option("concept_map", help="Type of diagram"),
    output: Optional[str] = typer.Option(None, help="Output file path")
):
    """Generate an educational diagram."""
    async def _run():
        agent = EducationalContentAgent()
        request = ContentRequest(
            content_type="diagram",
            concept=concept,
            diagram_type=diagram_type
        )
        result = await agent.process(request)

        if result.success:
            console.print(Panel(
                f"[green]Diagram generated successfully![/green]\n"
                f"URL: {result.data.get('url')}\n"
                f"Title: {result.data.get('title')}",
                title="Educational Content Agent"
            ))
        else:
            console.print(f"[red]Error:[/red] {result.error}")

    asyncio.run(_run())


@edu_app.command("gif")
def generate_gif(
    title: str = typer.Argument(..., help="Title of the animation"),
    steps: str = typer.Argument(..., help="Steps separated by semicolons")
):
    """Generate an animated GIF."""
    async def _run():
        agent = EducationalContentAgent()
        step_list = [s.strip() for s in steps.split(";")]
        request = ContentRequest(
            content_type="gif",
            concept=title,
            steps=step_list
        )
        result = await agent.process(request)

        if result.success:
            console.print(Panel(
                f"[green]GIF generated![/green]\n"
                f"URL: {result.data.get('url')}\n"
                f"Steps: {result.data.get('step_count')}\n"
                f"Duration: {result.data.get('duration_seconds'):.1f}s",
                title="Educational Content Agent"
            ))
        else:
            console.print(f"[red]Error:[/red] {result.error}")

    asyncio.run(_run())


@edu_app.command("summary")
def generate_summary(
    content: str = typer.Argument(..., help="Content to summarize"),
    max_words: int = typer.Option(400, help="Maximum words")
):
    """Generate a summary."""
    async def _run():
        agent = EducationalContentAgent()
        request = ContentRequest(
            content_type="summary",
            content=content
        )
        result = await agent.process(request)

        if result.success:
            console.print(Panel(
                Markdown(f"## Summary\n{result.data.get('summary')}\n\n"
                        f"### Key Points\n" +
                        "\n".join([f"- {p}" for p in result.data.get('key_points', [])])),
                title="Educational Content Agent"
            ))
        else:
            console.print(f"[red]Error:[/red] {result.error}")

    asyncio.run(_run())


@edu_app.command("types")
def list_diagram_types():
    """List available diagram types."""
    agent = EducationalContentAgent()
    types = agent.get_diagram_types()

    table = Table(title="Available Diagram Types")
    table.add_column("Type", style="cyan")
    table.add_column("Description")
    table.add_column("Best For", style="green")

    for type_id, info in types.items():
        table.add_row(
            type_id,
            info["description"],
            ", ".join(info["best_for"][:2])
        )

    console.print(table)


# RAG Commands
rag_app = typer.Typer(help="Multimodal RAG Agent commands")
app.add_typer(rag_app, name="rag")


@rag_app.command("chat")
def rag_chat(
    message: str = typer.Argument(..., help="Your question"),
    session: Optional[str] = typer.Option(None, help="Session ID")
):
    """Ask a question to the RAG agent."""
    async def _run():
        agent = MultimodalRAGAgent()
        request = ChatRequest(message=message, session_id=session)
        result = await agent.process(request)

        if result.success:
            console.print(Panel(
                Markdown(result.data.get("response", "")),
                title="RAG Agent Response"
            ))

            sources = result.data.get("sources", [])
            if sources:
                console.print("\n[dim]Sources:[/dim]")
                for s in sources:
                    console.print(f"  - {s.get('chapter_id')}: {s.get('section')}")

            console.print(f"\n[dim]Session: {result.data.get('session_id')}[/dim]")
        else:
            console.print(f"[red]Error:[/red] {result.error}")

    asyncio.run(_run())


@rag_app.command("search")
def rag_search(
    query: str = typer.Argument(..., help="Search query"),
    search_type: str = typer.Option("text", help="text, image, or hybrid")
):
    """Search the knowledge base."""
    async def _run():
        agent = MultimodalRAGAgent()

        if search_type == "text":
            results = await agent._search_text(query)
        elif search_type == "image":
            results = await agent._search_images(query)
        else:
            results = await agent._hybrid_search(query)
            results = [{"id": r.id, "content": r.content, "score": r.score} for r in results]

        if results:
            table = Table(title=f"Search Results ({search_type})")
            table.add_column("Score", style="cyan")
            table.add_column("Content")

            for r in results[:5]:
                score = f"{r.get('score', 0):.3f}"
                content = r.get("content", "")[:100] + "..."
                table.add_row(score, content)

            console.print(table)
        else:
            console.print("[yellow]No results found[/yellow]")

    asyncio.run(_run())


# SDD Commands
sdd_app = typer.Typer(help="SDD Planning Agent commands")
app.add_typer(sdd_app, name="sdd")


@sdd_app.command("specify")
def sdd_specify(
    description: str = typer.Argument(..., help="Feature description"),
    name: Optional[str] = typer.Option(None, help="Feature name")
):
    """Create a feature specification."""
    async def _run():
        agent = SDDPlanningAgent()
        request = SDDRequest(
            command="specify",
            description=description,
            feature=name
        )
        result = await agent.process(request)

        if result.success:
            console.print(Panel(
                f"[green]Specification created![/green]\n"
                f"File: {result.data.get('filepath')}",
                title="SDD Planning Agent"
            ))
        else:
            console.print(f"[red]Error:[/red] {result.error}")

    asyncio.run(_run())


@sdd_app.command("plan")
def sdd_plan(
    feature: str = typer.Argument(..., help="Feature branch name")
):
    """Generate implementation plan."""
    async def _run():
        agent = SDDPlanningAgent()
        request = SDDRequest(command="plan", feature=feature)
        result = await agent.process(request)

        if result.success:
            console.print(Panel(
                f"[green]Plan generated![/green]\n"
                f"File: {result.data.get('filepath')}",
                title="SDD Planning Agent"
            ))

            adr_suggestions = result.data.get("adr_suggestions", [])
            if adr_suggestions:
                console.print("\n[yellow]ADR Suggestions:[/yellow]")
                for s in adr_suggestions:
                    console.print(f"  Architectural decision detected: {s}")
                    console.print(f"  Run: sdd adr \"{s}\"")
        else:
            console.print(f"[red]Error:[/red] {result.error}")

    asyncio.run(_run())


@sdd_app.command("tasks")
def sdd_tasks(
    feature: str = typer.Argument(..., help="Feature branch name")
):
    """Generate implementation tasks."""
    async def _run():
        agent = SDDPlanningAgent()
        request = SDDRequest(command="tasks", feature=feature)
        result = await agent.process(request)

        if result.success:
            console.print(Panel(
                f"[green]Tasks generated![/green]\n"
                f"File: {result.data.get('filepath')}\n"
                f"Task count: {result.data.get('task_count')}",
                title="SDD Planning Agent"
            ))
        else:
            console.print(f"[red]Error:[/red] {result.error}")

    asyncio.run(_run())


@sdd_app.command("adr")
def sdd_adr(
    title: str = typer.Argument(..., help="Decision title"),
    context: str = typer.Option(..., help="Decision context"),
    decision: str = typer.Option(..., help="The decision made"),
    feature: Optional[str] = typer.Option(None, help="Related feature")
):
    """Create an Architecture Decision Record."""
    async def _run():
        agent = SDDPlanningAgent()
        request = SDDRequest(
            command="adr",
            title=title,
            context=context,
            content=decision,
            feature=feature
        )
        result = await agent.process(request)

        if result.success:
            console.print(Panel(
                f"[green]ADR created![/green]\n"
                f"File: {result.data.get('filepath')}\n"
                f"ADR ID: {result.data.get('adr_id')}",
                title="SDD Planning Agent"
            ))
        else:
            console.print(f"[red]Error:[/red] {result.error}")

    asyncio.run(_run())


# Orchestrator Commands
@app.command("ask")
def orchestrator_ask(
    message: str = typer.Argument(..., help="Your request")
):
    """Ask the orchestrator (auto-routes to appropriate agent)."""
    async def _run():
        orchestrator = OrchestratorAgent()
        request = OrchestratorRequest(message=message)
        result = await orchestrator.process(request)

        if result.success:
            console.print(Panel(
                json.dumps(result.data, indent=2, default=str),
                title="Orchestrator Response"
            ))
        else:
            console.print(f"[red]Error:[/red] {result.error}")

    asyncio.run(_run())


@app.command("capabilities")
def list_capabilities():
    """List all agent capabilities."""
    orchestrator = OrchestratorAgent()
    caps = orchestrator.get_capabilities()

    table = Table(title="Agent Capabilities")
    table.add_column("Agent", style="cyan")
    table.add_column("Capability")

    for cap in caps:
        if ":" in cap:
            agent, capability = cap.split(":", 1)
        else:
            agent, capability = "orchestrator", cap
        table.add_row(agent, capability)

    console.print(table)


@app.command("version")
def show_version():
    """Show version information."""
    from . import __version__
    console.print(f"Reusable Intelligence Agents v{__version__}")


if __name__ == "__main__":
    app()
