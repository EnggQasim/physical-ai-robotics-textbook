"""Tools for Educational Content Agent."""
import base64
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional
from dataclasses import dataclass

import google.generativeai as genai
from openai import AsyncOpenAI
from PIL import Image, ImageDraw, ImageFont

from ..shared.config import get_settings, DESIGN_GUIDELINES, DIAGRAM_TYPES


@dataclass
class DiagramResult:
    """Result from diagram generation."""
    success: bool
    url: Optional[str] = None
    title: Optional[str] = None
    diagram_type: Optional[str] = None
    cached: bool = False
    error: Optional[str] = None


@dataclass
class GifResult:
    """Result from GIF generation."""
    success: bool
    url: Optional[str] = None
    frames: List[Dict[str, Any]] = None
    step_count: int = 0
    duration_seconds: float = 0
    cached: bool = False
    error: Optional[str] = None


@dataclass
class SummaryResult:
    """Result from summary generation."""
    success: bool
    summary: Optional[str] = None
    key_points: List[str] = None
    main_concepts: List[str] = None
    word_count: int = 0
    cached: bool = False
    error: Optional[str] = None


@dataclass
class MindMapResult:
    """Result from mind map generation."""
    success: bool
    nodes: List[Dict[str, Any]] = None
    edges: List[Dict[str, Any]] = None
    central_topic: Optional[str] = None
    cached: bool = False
    error: Optional[str] = None


class DiagramGenerator:
    """Generate educational diagrams using Google Gemini."""

    def __init__(self, cache_dir: Optional[str] = None):
        settings = get_settings()
        self.cache_dir = Path(cache_dir or settings.cache_dir) / "diagrams"
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.cache_file = self.cache_dir / "cache.json"
        self.cache = self._load_cache()

        if settings.gemini_api_key:
            genai.configure(api_key=settings.gemini_api_key)
            self.model = genai.GenerativeModel(settings.gemini_model)
        else:
            self.model = None

    def _load_cache(self) -> Dict[str, Any]:
        if self.cache_file.exists():
            with open(self.cache_file, "r") as f:
                return json.load(f)
        return {}

    def _save_cache(self):
        with open(self.cache_file, "w") as f:
            json.dump(self.cache, f, indent=2)

    def _get_cache_key(self, prompt: str) -> str:
        return hashlib.md5(prompt.encode()).hexdigest()[:12]

    def _build_prompt(
        self,
        concept: str,
        diagram_type: str,
        custom_instructions: Optional[str] = None
    ) -> str:
        """Build a prompt for diagram generation."""
        type_info = DIAGRAM_TYPES.get(diagram_type, DIAGRAM_TYPES["concept_map"])
        colors = DESIGN_GUIDELINES["color_scheme"]

        prompt = f"""Create an educational {diagram_type.upper()} diagram for: {concept}

Diagram Type: {type_info['description']}
Best for: {', '.join(type_info['best_for'])}
Style: {type_info['style']}

Visual Requirements:
- Clean white background
- Primary color: {colors['primary']} (main concepts)
- Secondary color: {colors['secondary']} (processes/actions)
- Accent color: {colors['accent']} (highlights)
- Minimum font size: {DESIGN_GUIDELINES['accessibility']['font_size_min']}px
- Clear labels on all elements
- Professional technical style

Educational Best Practices:
- Maximum 8 elements for clarity
- Clear visual hierarchy
- Labeled connections/arrows
- Consistent iconography
"""
        if custom_instructions:
            prompt += f"\nAdditional Instructions:\n{custom_instructions}"

        return prompt

    async def generate(
        self,
        concept: str,
        diagram_type: str = "concept_map",
        custom_instructions: Optional[str] = None,
        force_regenerate: bool = False
    ) -> DiagramResult:
        """Generate a diagram for a concept."""
        if not self.model:
            return DiagramResult(
                success=False,
                error="Gemini API not configured. Set GEMINI_API_KEY."
            )

        prompt = self._build_prompt(concept, diagram_type, custom_instructions)
        cache_key = self._get_cache_key(prompt)

        # Check cache
        if not force_regenerate and cache_key in self.cache:
            cached = self.cache[cache_key]
            return DiagramResult(
                success=True,
                url=cached["url"],
                title=cached["title"],
                diagram_type=diagram_type,
                cached=True
            )

        try:
            response = self.model.generate_content(
                [prompt, "Generate this as a clean, professional diagram."],
                generation_config=genai.types.GenerationConfig(temperature=0.7)
            )

            # Check for image in response
            if response.candidates and response.candidates[0].content.parts:
                for part in response.candidates[0].content.parts:
                    if hasattr(part, 'inline_data') and part.inline_data:
                        image_data = part.inline_data.data
                        mime_type = part.inline_data.mime_type

                        ext = "png" if "png" in mime_type else "jpg"
                        filename = f"{cache_key}.{ext}"
                        filepath = self.cache_dir / filename

                        with open(filepath, "wb") as f:
                            data = base64.b64decode(image_data) if isinstance(image_data, str) else image_data
                            f.write(data)

                        url = str(filepath)
                        title = f"{concept} - {diagram_type.replace('_', ' ').title()}"

                        self.cache[cache_key] = {
                            "url": url,
                            "title": title,
                            "diagram_type": diagram_type
                        }
                        self._save_cache()

                        return DiagramResult(
                            success=True,
                            url=url,
                            title=title,
                            diagram_type=diagram_type,
                            cached=False
                        )

            return DiagramResult(
                success=False,
                error="No image generated - Gemini returned text only"
            )

        except Exception as e:
            return DiagramResult(success=False, error=str(e))

    def get_available_types(self) -> Dict[str, Dict]:
        """Get available diagram types."""
        return DIAGRAM_TYPES


class GifGenerator:
    """Generate animated educational GIFs."""

    def __init__(self, cache_dir: Optional[str] = None):
        settings = get_settings()
        self.cache_dir = Path(cache_dir or settings.cache_dir) / "gifs"
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.timing = DESIGN_GUIDELINES["animation_timing"]
        self.colors = DESIGN_GUIDELINES["color_scheme"]

    async def generate(
        self,
        title: str,
        steps: List[str],
        output_filename: Optional[str] = None,
        width: int = 800,
        height: int = 600,
        fps: int = 5
    ) -> GifResult:
        """Generate an animated GIF from steps."""
        if len(steps) < 2:
            return GifResult(success=False, error="At least 2 steps required")

        if len(steps) > self.timing["max_steps"]:
            return GifResult(
                success=False,
                error=f"Maximum {self.timing['max_steps']} steps allowed"
            )

        try:
            frames = []
            frames_per_step = int((self.timing["step_duration_ms"] / 1000) * fps)

            for step_idx, step_text in enumerate(steps):
                frame = self._create_frame(
                    title=title,
                    step_text=step_text,
                    step_num=step_idx + 1,
                    total_steps=len(steps),
                    width=width,
                    height=height
                )

                # Add multiple copies for duration
                for _ in range(frames_per_step):
                    frames.append(frame)

            # Save GIF
            if not output_filename:
                cache_key = hashlib.md5(f"{title}{''.join(steps)}".encode()).hexdigest()[:12]
                output_filename = f"{cache_key}.gif"

            filepath = self.cache_dir / output_filename

            frames[0].save(
                filepath,
                save_all=True,
                append_images=frames[1:],
                duration=int(1000 / fps),
                loop=0,
                optimize=True
            )

            duration_seconds = len(frames) / fps

            return GifResult(
                success=True,
                url=str(filepath),
                frames=[{"step": i+1, "description": s} for i, s in enumerate(steps)],
                step_count=len(steps),
                duration_seconds=duration_seconds
            )

        except Exception as e:
            return GifResult(success=False, error=str(e))

    def _create_frame(
        self,
        title: str,
        step_text: str,
        step_num: int,
        total_steps: int,
        width: int,
        height: int
    ) -> Image.Image:
        """Create a single frame for the GIF."""
        # Create image with white background
        img = Image.new('RGB', (width, height), 'white')
        draw = ImageDraw.Draw(img)

        # Try to load a font, fall back to default
        try:
            title_font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 32)
            step_font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 24)
            small_font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 16)
        except:
            title_font = ImageFont.load_default()
            step_font = ImageFont.load_default()
            small_font = ImageFont.load_default()

        # Draw title
        draw.text((width // 2, 40), title, fill=self.colors["primary"], font=title_font, anchor="mm")

        # Draw step indicator
        indicator_text = f"Step {step_num} of {total_steps}"
        draw.text((width // 2, 80), indicator_text, fill=self.colors["secondary"], font=small_font, anchor="mm")

        # Draw step content in center
        draw.text((width // 2, height // 2), step_text, fill=self.colors["neutral"], font=step_font, anchor="mm")

        # Draw progress bar at bottom
        bar_width = width - 100
        bar_height = 20
        bar_x = 50
        bar_y = height - 60

        # Background bar
        draw.rectangle([bar_x, bar_y, bar_x + bar_width, bar_y + bar_height], outline=self.colors["neutral"])

        # Progress fill
        progress = step_num / total_steps
        fill_width = int(bar_width * progress)
        draw.rectangle(
            [bar_x, bar_y, bar_x + fill_width, bar_y + bar_height],
            fill=self.colors["success"]
        )

        # Draw step dots
        dot_y = height - 30
        dot_spacing = bar_width / (total_steps + 1)
        for i in range(total_steps):
            dot_x = bar_x + dot_spacing * (i + 1)
            color = self.colors["success"] if i < step_num else self.colors["neutral"]
            draw.ellipse([dot_x - 5, dot_y - 5, dot_x + 5, dot_y + 5], fill=color)

        return img


class SummaryGenerator:
    """Generate educational summaries using OpenAI."""

    def __init__(self):
        settings = get_settings()
        if settings.openai_api_key:
            self.client = AsyncOpenAI(api_key=settings.openai_api_key)
            self.model = settings.openai_chat_model
        else:
            self.client = None

    async def generate(
        self,
        content: str,
        max_words: int = 400,
        min_words: int = 200
    ) -> SummaryResult:
        """Generate a summary of content."""
        if not self.client:
            return SummaryResult(
                success=False,
                error="OpenAI API not configured. Set OPENAI_API_KEY."
            )

        prompt = f"""Summarize the following educational content.

Requirements:
- Length: {min_words}-{max_words} words
- Structure: Start with a brief overview, then key points
- Include: Main concepts, key takeaways, important terms
- Tone: Clear, educational, accessible

Content to summarize:
{content}

Respond in JSON format:
{{
    "summary": "The main summary text...",
    "key_points": ["Point 1", "Point 2", ...],
    "main_concepts": ["Concept 1", "Concept 2", ...]
}}
"""

        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"}
            )

            result = json.loads(response.choices[0].message.content)

            return SummaryResult(
                success=True,
                summary=result.get("summary", ""),
                key_points=result.get("key_points", []),
                main_concepts=result.get("main_concepts", []),
                word_count=len(result.get("summary", "").split())
            )

        except Exception as e:
            return SummaryResult(success=False, error=str(e))


class MindMapGenerator:
    """Generate mind map structures using OpenAI."""

    def __init__(self):
        settings = get_settings()
        if settings.openai_api_key:
            self.client = AsyncOpenAI(api_key=settings.openai_api_key)
            self.model = settings.openai_chat_model
        else:
            self.client = None

    async def generate(
        self,
        content: str,
        central_topic: Optional[str] = None,
        max_nodes: int = 15,
        max_depth: int = 3
    ) -> MindMapResult:
        """Generate a mind map structure from content."""
        if not self.client:
            return MindMapResult(
                success=False,
                error="OpenAI API not configured. Set OPENAI_API_KEY."
            )

        prompt = f"""Analyze the following content and create a mind map structure.

Requirements:
- Central topic: {central_topic or 'Identify from content'}
- Maximum nodes: {max_nodes}
- Maximum depth: {max_depth} levels
- Each node needs: id, label, description, level (0=center, 1=main branch, 2=sub-branch)
- Include edges connecting parent-child nodes

Content:
{content}

Respond in JSON format:
{{
    "central_topic": "Main Topic",
    "nodes": [
        {{"id": "1", "label": "Main Topic", "description": "Brief description", "level": 0}},
        {{"id": "2", "label": "Branch 1", "description": "Description", "level": 1, "parent_id": "1"}},
        ...
    ],
    "edges": [
        {{"source": "1", "target": "2", "relationship": "includes"}},
        ...
    ]
}}
"""

        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"}
            )

            result = json.loads(response.choices[0].message.content)

            return MindMapResult(
                success=True,
                nodes=result.get("nodes", []),
                edges=result.get("edges", []),
                central_topic=result.get("central_topic", central_topic)
            )

        except Exception as e:
            return MindMapResult(success=False, error=str(e))
