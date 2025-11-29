#!/usr/bin/env python3
"""
Gemini Image Generation Script
Generates conceptual diagrams and visualizations using Google Gemini API.

Usage:
    python gemini_image.py --prompt "ROS2 node communication diagram" --output diagram.png
    python gemini_image.py --prompt "Robot arm kinematics" --output arm.png --style "technical"
"""

import argparse
import base64
import json
import os
import sys
from pathlib import Path
from datetime import datetime

try:
    import google.generativeai as genai
    from PIL import Image
    import io
except ImportError:
    print("ERROR: Required packages not installed. Run:")
    print("  pip install google-generativeai pillow")
    sys.exit(1)


# Default API key (can be overridden by environment variable)
DEFAULT_API_KEY = "AIzaSyDQ3DaEOvDbnPw6xRI_s5R-vwyW-QqiU5g"


def setup_gemini(api_key: str = None, model_name: str = None):
    """Configure the Gemini API."""
    key = api_key or os.environ.get("GEMINI_API_KEY", DEFAULT_API_KEY)
    genai.configure(api_key=key)
    # Try different models in order of preference
    model = model_name or os.environ.get("GEMINI_MODEL", "gemini-2.0-flash")
    return genai.GenerativeModel(model)


def generate_image_prompt(concept: str, style: str = "technical") -> str:
    """Generate an optimized prompt for diagram generation."""
    style_guides = {
        "technical": "Clean technical diagram with labeled components, arrows showing data flow, use NVIDIA green (#76b900) as accent color, white background, professional engineering style",
        "workflow": "Flowchart style diagram with clear boxes and arrows, sequential steps numbered, use green (#76b900) highlights, clean minimal design",
        "architecture": "System architecture diagram showing components as boxes, connections as lines, hierarchical layout, technical documentation style, green (#76b900) accent",
        "animated": "Frame sequence showing motion or process steps, clear progression indicators, suitable for GIF animation",
        "simple": "Minimalist diagram with basic shapes, easy to understand, educational style, suitable for beginners",
    }

    style_guide = style_guides.get(style, style_guides["technical"])

    return f"""Create a clear, professional diagram for: {concept}

Style requirements:
- {style_guide}
- No text watermarks or signatures
- High contrast for readability
- Suitable for technical documentation
- Clean lines and professional appearance

The diagram should clearly illustrate the concept for educational purposes."""


def generate_diagram(
    prompt: str,
    output_path: str,
    style: str = "technical",
    width: int = 800,
    height: int = 600,
) -> dict:
    """
    Generate a diagram using Gemini API.

    Args:
        prompt: Description of the diagram to generate
        output_path: Path to save the generated image
        style: Diagram style (technical, workflow, architecture, animated, simple)
        width: Image width in pixels
        height: Image height in pixels

    Returns:
        dict with status, path, and metadata
    """
    try:
        model = setup_gemini()

        # Generate optimized prompt
        full_prompt = generate_image_prompt(prompt, style)

        # For now, Gemini doesn't directly generate images via the Python SDK
        # We'll use it to generate SVG code or detailed diagram descriptions
        # that can be rendered

        svg_prompt = f"""Generate SVG code for a diagram illustrating: {prompt}

Requirements:
- Use viewBox="0 0 {width} {height}"
- Primary color: #76b900 (NVIDIA green)
- Secondary colors: #1a1a1a (dark), #ffffff (white), #666666 (gray)
- Include clear labels with font-family="Arial, sans-serif"
- Use clean lines and professional styling
- Add appropriate arrows for flow/connections
- Make it suitable for technical documentation

Return ONLY valid SVG code, no explanations. Start with <svg and end with </svg>."""

        response = model.generate_content(svg_prompt)
        svg_content = response.text.strip()

        # Clean up SVG content
        if "```svg" in svg_content:
            svg_content = svg_content.split("```svg")[1].split("```")[0].strip()
        elif "```xml" in svg_content:
            svg_content = svg_content.split("```xml")[1].split("```")[0].strip()
        elif "```" in svg_content:
            svg_content = svg_content.split("```")[1].split("```")[0].strip()

        # Ensure it starts with <svg
        if not svg_content.startswith("<svg"):
            # Try to find SVG content
            start = svg_content.find("<svg")
            if start != -1:
                end = svg_content.rfind("</svg>") + 6
                svg_content = svg_content[start:end]
            else:
                return {
                    "status": "error",
                    "message": "Failed to generate valid SVG content",
                    "raw_response": response.text[:500]
                }

        # Save as SVG
        output_path = Path(output_path)
        if output_path.suffix.lower() != ".svg":
            svg_path = output_path.with_suffix(".svg")
        else:
            svg_path = output_path

        # Ensure output directory exists
        svg_path.parent.mkdir(parents=True, exist_ok=True)

        with open(svg_path, "w") as f:
            f.write(svg_content)

        return {
            "status": "success",
            "path": str(svg_path),
            "format": "svg",
            "prompt": prompt,
            "style": style,
            "timestamp": datetime.now().isoformat(),
            "size": len(svg_content),
        }

    except Exception as e:
        return {
            "status": "error",
            "message": str(e),
            "prompt": prompt,
        }


def generate_description(concept: str) -> dict:
    """
    Generate a detailed description of a diagram that could be created.
    Useful for planning or when image generation fails.

    Args:
        concept: The concept to describe

    Returns:
        dict with description and ASCII diagram
    """
    try:
        model = setup_gemini()

        prompt = f"""For the concept: {concept}

Provide:
1. A brief description of what a diagram should show (2-3 sentences)
2. An ASCII art representation of the diagram
3. Key components that should be labeled

Format your response as:
DESCRIPTION: <your description>

ASCII:
```
<ascii diagram here>
```

LABELS: <comma-separated list of key labels>"""

        response = model.generate_content(prompt)

        return {
            "status": "success",
            "concept": concept,
            "response": response.text,
            "type": "description",
        }

    except Exception as e:
        return {
            "status": "error",
            "message": str(e),
            "concept": concept,
        }


def main():
    parser = argparse.ArgumentParser(
        description="Generate diagrams using Google Gemini API"
    )
    parser.add_argument(
        "--prompt", "-p",
        required=True,
        help="Description of the diagram to generate"
    )
    parser.add_argument(
        "--output", "-o",
        default="diagram.svg",
        help="Output file path (default: diagram.svg)"
    )
    parser.add_argument(
        "--style", "-s",
        choices=["technical", "workflow", "architecture", "animated", "simple"],
        default="technical",
        help="Diagram style (default: technical)"
    )
    parser.add_argument(
        "--width", "-W",
        type=int,
        default=800,
        help="Image width in pixels (default: 800)"
    )
    parser.add_argument(
        "--height", "-H",
        type=int,
        default=600,
        help="Image height in pixels (default: 600)"
    )
    parser.add_argument(
        "--describe-only", "-d",
        action="store_true",
        help="Only generate description, no image"
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output result as JSON"
    )
    parser.add_argument(
        "--api-key", "-k",
        help="Gemini API key (overrides default and environment variable)"
    )
    parser.add_argument(
        "--model", "-m",
        default="gemini-2.0-flash",
        help="Gemini model to use (default: gemini-2.0-flash)"
    )

    args = parser.parse_args()

    if args.api_key:
        os.environ["GEMINI_API_KEY"] = args.api_key

    if args.model:
        os.environ["GEMINI_MODEL"] = args.model

    if args.describe_only:
        result = generate_description(args.prompt)
    else:
        result = generate_diagram(
            prompt=args.prompt,
            output_path=args.output,
            style=args.style,
            width=args.width,
            height=args.height,
        )

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        if result["status"] == "success":
            if "path" in result:
                print(f"✓ Generated: {result['path']}")
                print(f"  Style: {result.get('style', 'N/A')}")
                print(f"  Size: {result.get('size', 'N/A')} bytes")
            else:
                print(f"✓ Description generated for: {result['concept']}")
                print(result.get("response", ""))
        else:
            print(f"✗ Error: {result.get('message', 'Unknown error')}")
            sys.exit(1)


if __name__ == "__main__":
    main()
