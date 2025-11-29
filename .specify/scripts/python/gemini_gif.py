#!/usr/bin/env python3
"""
Gemini GIF Generator for Physical AI Textbook

Generates animated GIF images using Google Gemini 2.5 Flash (Nano Banana) API
to visualize complex robotics and AI concepts through simple animations.

Usage:
    python gemini_gif.py --prompt "ROS2 pub/sub message flow" --output output.gif
    python gemini_gif.py --prompt "robot motion" --style loop --duration medium
"""

import argparse
import json
import os
import sys
from pathlib import Path
from datetime import datetime

try:
    import google.generativeai as genai
    from PIL import Image
    import io
    import base64
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Install with: pip install google-generativeai pillow")
    sys.exit(1)


# Theme colors for NVIDIA-style animations
THEME = {
    "primary": "#76b900",  # NVIDIA green
    "dark": "#1a1a1a",
    "light": "#ffffff",
    "gray": "#666666",
}

# Animation style prompts
STYLE_PROMPTS = {
    "loop": "Create a seamlessly looping animation where the end connects smoothly to the beginning.",
    "sequence": "Create a step-by-step sequence animation showing each stage clearly.",
    "flow": "Create an animation showing data or messages flowing through the system.",
    "transition": "Create an animation showing smooth transitions between states.",
    "cycle": "Create a cyclical animation showing a repeating process.",
}

# Duration settings
DURATION_SETTINGS = {
    "short": {"frames": 8, "fps": 10, "description": "2 seconds"},
    "medium": {"frames": 16, "fps": 12, "description": "4 seconds"},
    "long": {"frames": 24, "fps": 15, "description": "6 seconds"},
}


def setup_gemini(api_key: str = None, model_name: str = None):
    """Initialize the Gemini API client."""
    key = api_key or os.environ.get("GEMINI_API_KEY")
    if not key:
        raise ValueError(
            "GEMINI_API_KEY environment variable is required.\n"
            "Set it with: export GEMINI_API_KEY='your-key-here'"
        )

    genai.configure(api_key=key)

    # Use Gemini 2.5 Flash for image/GIF generation (Nano Banana)
    model = model_name or "gemini-2.0-flash-exp"
    return genai.GenerativeModel(model)


def build_animation_prompt(concept: str, style: str = "loop", duration: str = "medium") -> str:
    """Build a detailed prompt for GIF generation."""

    style_instruction = STYLE_PROMPTS.get(style, STYLE_PROMPTS["loop"])
    duration_info = DURATION_SETTINGS.get(duration, DURATION_SETTINGS["medium"])

    prompt = f"""Generate an animated GIF illustration for a technical textbook about Physical AI and Robotics.

CONCEPT TO ANIMATE:
{concept}

ANIMATION STYLE:
{style_instruction}

VISUAL REQUIREMENTS:
- Color scheme: Use NVIDIA green ({THEME['primary']}) as the primary accent color
- Background: Dark ({THEME['dark']}) or clean white ({THEME['light']})
- Style: Clean, minimalist technical illustration
- Labels: Include clear text labels for key components
- Frames: Approximately {duration_info['frames']} frames at {duration_info['fps']} fps ({duration_info['description']})

TECHNICAL STYLE:
- Use simple geometric shapes (circles, rectangles, arrows)
- Show movement with motion lines or trails
- Highlight active elements with glow or pulse effects
- Keep the animation focused on ONE main concept
- Ensure smooth transitions between frames

OUTPUT:
Generate an animated GIF that clearly demonstrates this concept for students learning robotics.
The animation should be educational and easy to understand at a glance.
"""
    return prompt


def generate_gif(
    prompt: str,
    output_path: str,
    style: str = "loop",
    duration: str = "medium",
    model: genai.GenerativeModel = None,
    verbose: bool = False
) -> dict:
    """Generate an animated GIF using Gemini API."""

    if model is None:
        model = setup_gemini()

    full_prompt = build_animation_prompt(prompt, style, duration)

    if verbose:
        print(f"Generating GIF with prompt:\n{full_prompt[:200]}...")

    try:
        # Request image generation with animation capability
        response = model.generate_content(
            full_prompt,
            generation_config={
                "temperature": 0.7,
                "top_p": 0.95,
            }
        )

        # Check if we got an image response
        if response.candidates and response.candidates[0].content.parts:
            for part in response.candidates[0].content.parts:
                if hasattr(part, 'inline_data') and part.inline_data:
                    # Save the generated image/GIF
                    image_data = base64.b64decode(part.inline_data.data)

                    # Ensure output directory exists
                    output_file = Path(output_path)
                    output_file.parent.mkdir(parents=True, exist_ok=True)

                    with open(output_file, 'wb') as f:
                        f.write(image_data)

                    return {
                        "success": True,
                        "output_path": str(output_file.absolute()),
                        "mime_type": part.inline_data.mime_type,
                        "size_bytes": len(image_data),
                        "prompt": prompt,
                        "style": style,
                        "duration": duration,
                    }

        # If no image in response, return the text (might be an explanation)
        text_response = response.text if hasattr(response, 'text') else str(response)
        return {
            "success": False,
            "error": "No image generated",
            "message": text_response[:500],
            "prompt": prompt,
        }

    except Exception as e:
        return {
            "success": False,
            "error": str(e),
            "prompt": prompt,
        }


def create_placeholder_gif(output_path: str, concept: str) -> dict:
    """Create a placeholder GIF when API generation fails."""
    try:
        from PIL import Image, ImageDraw, ImageFont

        # Create frames for a simple placeholder animation
        frames = []
        width, height = 600, 400

        for i in range(8):
            img = Image.new('RGB', (width, height), THEME['dark'])
            draw = ImageDraw.Draw(img)

            # Draw border
            draw.rectangle([10, 10, width-10, height-10], outline=THEME['primary'], width=2)

            # Draw text
            text = f"Animation: {concept[:30]}..."
            draw.text((width//2, height//2 - 20), text, fill=THEME['primary'], anchor="mm")

            # Draw animated element (moving dot)
            dot_x = 50 + (i * (width - 100) // 8)
            draw.ellipse([dot_x-10, height//2+30, dot_x+10, height//2+50], fill=THEME['primary'])

            # Draw frame counter
            draw.text((width-30, height-30), f"{i+1}/8", fill=THEME['gray'])

            frames.append(img)

        # Save as GIF
        output_file = Path(output_path)
        output_file.parent.mkdir(parents=True, exist_ok=True)

        frames[0].save(
            output_file,
            save_all=True,
            append_images=frames[1:],
            duration=200,
            loop=0
        )

        return {
            "success": True,
            "output_path": str(output_file.absolute()),
            "placeholder": True,
            "message": "Created placeholder GIF (API unavailable)",
        }

    except Exception as e:
        return {
            "success": False,
            "error": f"Failed to create placeholder: {e}",
        }


def main():
    parser = argparse.ArgumentParser(
        description="Generate animated GIF images using Google Gemini API"
    )
    parser.add_argument(
        "--prompt", "-p",
        required=True,
        help="Description of the concept to animate"
    )
    parser.add_argument(
        "--output", "-o",
        default="output.gif",
        help="Output file path (default: output.gif)"
    )
    parser.add_argument(
        "--style", "-s",
        choices=["loop", "sequence", "flow", "transition", "cycle"],
        default="loop",
        help="Animation style (default: loop)"
    )
    parser.add_argument(
        "--duration", "-d",
        choices=["short", "medium", "long"],
        default="medium",
        help="Animation duration (default: medium)"
    )
    parser.add_argument(
        "--model", "-m",
        default=None,
        help="Gemini model to use (default: gemini-2.0-flash-exp)"
    )
    parser.add_argument(
        "--json", "-j",
        action="store_true",
        help="Output result as JSON"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Verbose output"
    )
    parser.add_argument(
        "--placeholder",
        action="store_true",
        help="Create placeholder GIF (for testing without API)"
    )

    args = parser.parse_args()

    if args.placeholder:
        result = create_placeholder_gif(args.output, args.prompt)
    else:
        try:
            model = setup_gemini(model_name=args.model)
            result = generate_gif(
                prompt=args.prompt,
                output_path=args.output,
                style=args.style,
                duration=args.duration,
                model=model,
                verbose=args.verbose,
            )
        except ValueError as e:
            # API key not set, create placeholder
            if args.verbose:
                print(f"API setup failed: {e}")
                print("Creating placeholder GIF instead...")
            result = create_placeholder_gif(args.output, args.prompt)
            result["api_error"] = str(e)

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        if result.get("success"):
            print(f"✅ GIF generated: {result['output_path']}")
            if result.get("placeholder"):
                print("   (placeholder - set GEMINI_API_KEY for real generation)")
        else:
            print(f"❌ Generation failed: {result.get('error', 'Unknown error')}")
            if result.get("message"):
                print(f"   Message: {result['message'][:200]}")

    return 0 if result.get("success") else 1


if __name__ == "__main__":
    sys.exit(main())
