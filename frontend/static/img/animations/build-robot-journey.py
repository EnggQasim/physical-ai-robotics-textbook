#!/usr/bin/env python3
"""
Generate animated GIF showing human learning from book and building a robot.
Stages:
1. Human reading Physical AI book
2. Human starts building robot (parts appear)
3. Robot assembly in progress
4. Robot completed and powered on
5. Robot working like a human (walking, waving)
"""

from PIL import Image, ImageDraw, ImageFont
import math
import os

# Configuration
WIDTH, HEIGHT = 800, 500
FRAMES = 60
FPS = 12
DURATION = int(1000 / FPS)

# Colors - NVIDIA theme
BG_COLOR = (26, 26, 26)  # #1a1a1a
GREEN = (118, 185, 0)    # #76b900
DARK_GREEN = (80, 130, 0)
WHITE = (255, 255, 255)
GRAY = (100, 100, 100)
DARK_GRAY = (50, 50, 50)
LIGHT_GRAY = (150, 150, 150)

def draw_human(draw, x, y, scale=1.0, reading=False, building=False, celebrating=False):
    """Draw a simple human figure"""
    s = scale
    # Head
    draw.ellipse([x-15*s, y-60*s, x+15*s, y-30*s], fill=LIGHT_GRAY, outline=WHITE, width=2)
    # Body
    draw.rectangle([x-20*s, y-30*s, x+20*s, y+30*s], fill=GRAY, outline=WHITE, width=2)
    # Legs
    draw.rectangle([x-18*s, y+30*s, x-5*s, y+70*s], fill=GRAY, outline=WHITE, width=2)
    draw.rectangle([x+5*s, y+30*s, x+18*s, y+70*s], fill=GRAY, outline=WHITE, width=2)

    if reading:
        # Arms holding book
        draw.rectangle([x-35*s, y-20*s, x-20*s, y+10*s], fill=GRAY, outline=WHITE, width=2)
        draw.rectangle([x+20*s, y-20*s, x+35*s, y+10*s], fill=GRAY, outline=WHITE, width=2)
    elif building:
        # Arms extended (working)
        draw.rectangle([x+20*s, y-25*s, x+60*s, y-10*s], fill=GRAY, outline=WHITE, width=2)
        draw.rectangle([x+20*s, y+0*s, x+55*s, y+15*s], fill=GRAY, outline=WHITE, width=2)
    elif celebrating:
        # Arms raised
        draw.rectangle([x-40*s, y-50*s, x-25*s, y-20*s], fill=GRAY, outline=WHITE, width=2)
        draw.rectangle([x+25*s, y-50*s, x+40*s, y-20*s], fill=GRAY, outline=WHITE, width=2)
    else:
        # Arms at side
        draw.rectangle([x-35*s, y-25*s, x-20*s, y+20*s], fill=GRAY, outline=WHITE, width=2)
        draw.rectangle([x+20*s, y-25*s, x+35*s, y+20*s], fill=GRAY, outline=WHITE, width=2)

def draw_book(draw, x, y, scale=1.0, open_amount=0.8):
    """Draw the Physical AI book"""
    s = scale
    w = 60 * s
    h = 80 * s

    # Book cover
    draw.rectangle([x-w/2, y-h/2, x+w/2, y+h/2], fill=DARK_GREEN, outline=GREEN, width=3)

    # Book title
    draw.rectangle([x-w/2+8, y-h/2+10, x+w/2-8, y-h/2+35], fill=BG_COLOR)

    # Pages (if open)
    if open_amount > 0:
        page_offset = int(10 * open_amount * s)
        draw.rectangle([x-w/2+5, y-h/2+5, x-w/2+5+page_offset, y+h/2-5], fill=WHITE)

def draw_robot(draw, x, y, scale=1.0, assembly_progress=1.0, powered=False, walking_frame=0):
    """Draw robot at various assembly stages"""
    s = scale

    # Assembly stages
    if assembly_progress < 0.2:
        return  # Nothing yet

    # Legs (first to appear)
    if assembly_progress >= 0.2:
        alpha = min(1.0, (assembly_progress - 0.2) / 0.15)
        leg_color = tuple(int(c * alpha) for c in GREEN)

        # Walking animation
        leg_offset = int(math.sin(walking_frame * 0.5) * 8) if powered else 0

        draw.rectangle([x-25*s, y+20*s, x-8*s, y+70*s+leg_offset], fill=BG_COLOR, outline=GREEN, width=2)
        draw.rectangle([x+8*s, y+20*s, x+25*s, y+70*s-leg_offset], fill=BG_COLOR, outline=GREEN, width=2)
        # Feet
        draw.rectangle([x-30*s, y+65*s+leg_offset, x-5*s, y+80*s+leg_offset], fill=DARK_GRAY, outline=GREEN, width=2)
        draw.rectangle([x+5*s, y+65*s-leg_offset, x+30*s, y+80*s-leg_offset], fill=DARK_GRAY, outline=GREEN, width=2)

    # Torso
    if assembly_progress >= 0.35:
        draw.rectangle([x-30*s, y-30*s, x+30*s, y+25*s], fill=BG_COLOR, outline=GREEN, width=2)
        # Chest panel
        draw.rectangle([x-20*s, y-20*s, x+20*s, y+10*s], fill=DARK_GRAY, outline=GREEN, width=1)
        if powered:
            # Glowing indicators
            for i, cx in enumerate([-12, 0, 12]):
                draw.ellipse([x+cx*s-4, y-10*s-4, x+cx*s+4, y-10*s+4], fill=GREEN)

    # Arms
    if assembly_progress >= 0.5:
        wave_offset = int(math.sin(walking_frame * 0.3) * 15) if powered else 0

        # Left arm
        draw.rectangle([x-50*s, y-25*s, x-30*s, y+15*s], fill=BG_COLOR, outline=GREEN, width=2)
        draw.rectangle([x-55*s, y+10*s, x-35*s, y+40*s-wave_offset], fill=BG_COLOR, outline=GREEN, width=2)

        # Right arm (waving if powered)
        if powered and walking_frame % 20 > 10:
            # Raised arm
            draw.rectangle([x+30*s, y-25*s, x+50*s, y+5*s], fill=BG_COLOR, outline=GREEN, width=2)
            draw.rectangle([x+35*s, y-55*s, x+55*s, y-20*s], fill=BG_COLOR, outline=GREEN, width=2)
            # Hand waving
            draw.ellipse([x+38*s, y-65*s, x+52*s, y-50*s], fill=DARK_GRAY, outline=GREEN, width=2)
        else:
            draw.rectangle([x+30*s, y-25*s, x+50*s, y+15*s], fill=BG_COLOR, outline=GREEN, width=2)
            draw.rectangle([x+35*s, y+10*s, x+55*s, y+40*s+wave_offset], fill=BG_COLOR, outline=GREEN, width=2)

    # Head (last to appear)
    if assembly_progress >= 0.7:
        # Neck
        draw.rectangle([x-10*s, y-40*s, x+10*s, y-28*s], fill=DARK_GRAY, outline=GREEN, width=1)
        # Head
        draw.rectangle([x-25*s, y-80*s, x+25*s, y-38*s], fill=BG_COLOR, outline=GREEN, width=2)
        # Face visor
        draw.rectangle([x-18*s, y-72*s, x+18*s, y-50*s], fill=DARK_GRAY, outline=GREEN, width=1)

        # Eyes
        if powered:
            eye_blink = walking_frame % 30 < 3
            eye_height = 2 if eye_blink else 6
            draw.ellipse([x-12*s-5, y-65*s-eye_height, x-12*s+5, y-65*s+eye_height], fill=GREEN)
            draw.ellipse([x+12*s-5, y-65*s-eye_height, x+12*s+5, y-65*s+eye_height], fill=GREEN)
            # Eye shine
            draw.ellipse([x-12*s-2, y-65*s-2, x-12*s+2, y-65*s+2], fill=WHITE)
            draw.ellipse([x+12*s-2, y-65*s-2, x+12*s+2, y-65*s+2], fill=WHITE)
        else:
            draw.ellipse([x-12*s-5, y-65*s-5, x-12*s+5, y-65*s+5], fill=DARK_GRAY)
            draw.ellipse([x+12*s-5, y-65*s-5, x+12*s+5, y-65*s+5], fill=DARK_GRAY)

        # Antenna
        draw.line([x, y-80*s, x, y-95*s], fill=GREEN, width=3)
        if powered:
            pulse = int(math.sin(walking_frame * 0.5) * 3) + 6
            draw.ellipse([x-pulse, y-100*s-pulse, x+pulse, y-100*s+pulse], fill=GREEN)
        else:
            draw.ellipse([x-5, y-100*s-5, x+5, y-100*s+5], fill=DARK_GRAY)

def draw_parts(draw, x, y, visible_parts):
    """Draw robot parts scattered (before assembly)"""
    parts = [
        ("Head", x-60, y-40, 40, 35),
        ("Arm", x+50, y-30, 15, 40),
        ("Arm", x+80, y+10, 15, 40),
        ("Torso", x-40, y+30, 50, 45),
        ("Leg", x+30, y+50, 18, 45),
        ("Leg", x+60, y+60, 18, 45),
    ]

    for i, (name, px, py, w, h) in enumerate(parts):
        if i < visible_parts:
            draw.rectangle([px-w/2, py-h/2, px+w/2, py+h/2], fill=DARK_GRAY, outline=GREEN, width=2)

def draw_text(draw, text, x, y, color=GREEN, size="normal"):
    """Draw text at position"""
    draw.text((x, y), text, fill=color, anchor="mm")

def draw_progress_bar(draw, x, y, width, progress, label=""):
    """Draw a progress bar"""
    height = 20
    # Background
    draw.rectangle([x, y, x+width, y+height], fill=DARK_GRAY, outline=GREEN, width=1)
    # Progress
    fill_width = int(width * progress)
    if fill_width > 0:
        draw.rectangle([x+2, y+2, x+2+fill_width-4, y+height-2], fill=GREEN)
    # Label
    if label:
        draw.text((x + width/2, y + height + 15), label, fill=WHITE, anchor="mm")

def create_frame(frame_num, total_frames):
    """Create a single frame of the animation"""
    img = Image.new('RGB', (WIDTH, HEIGHT), BG_COLOR)
    draw = ImageDraw.Draw(img)

    # Calculate animation phase (0-5)
    progress = frame_num / total_frames

    # Phase durations
    # 0.00-0.15: Reading book
    # 0.15-0.25: Transition to building
    # 0.25-0.60: Building robot (parts appearing, assembly)
    # 0.60-0.75: Robot powering on
    # 0.75-1.00: Robot working like human

    # Title
    draw.text((WIDTH/2, 30), "From Learning to Creation", fill=GREEN, anchor="mm")

    # Draw scene based on phase
    if progress < 0.15:
        # Phase 1: Reading
        phase_progress = progress / 0.15
        draw.text((WIDTH/2, HEIGHT-40), "Step 1: Learning Physical AI", fill=WHITE, anchor="mm")

        # Human reading book
        draw_human(draw, 250, 280, scale=1.2, reading=True)
        draw_book(draw, 250, 230, scale=1.5, open_amount=0.5 + phase_progress * 0.3)

        # Knowledge bubbles appearing
        if phase_progress > 0.3:
            draw.text((350, 150), "ROS2", fill=GREEN, anchor="mm")
        if phase_progress > 0.5:
            draw.text((400, 180), "Gazebo", fill=GREEN, anchor="mm")
        if phase_progress > 0.7:
            draw.text((380, 220), "Isaac", fill=GREEN, anchor="mm")
        if phase_progress > 0.9:
            draw.text((420, 250), "VLA", fill=GREEN, anchor="mm")

    elif progress < 0.25:
        # Phase 2: Transition
        phase_progress = (progress - 0.15) / 0.10
        draw.text((WIDTH/2, HEIGHT-40), "Step 2: Starting to Build", fill=WHITE, anchor="mm")

        # Human moving to building position
        human_x = 250 - int(100 * phase_progress)
        draw_human(draw, human_x, 280, scale=1.2, building=phase_progress > 0.5)

        # Book fading/moving
        if phase_progress < 0.5:
            draw_book(draw, 250, 230, scale=1.5 - phase_progress, open_amount=0.8)

        # Parts starting to appear
        draw_parts(draw, 500, 280, int(phase_progress * 3))

    elif progress < 0.60:
        # Phase 3: Building
        phase_progress = (progress - 0.25) / 0.35
        draw.text((WIDTH/2, HEIGHT-40), "Step 3: Assembling the Robot", fill=WHITE, anchor="mm")

        # Human building
        draw_human(draw, 150, 280, scale=1.2, building=True)

        # Robot being assembled
        assembly = min(1.0, phase_progress * 1.2)
        draw_robot(draw, 550, 300, scale=1.0, assembly_progress=assembly, powered=False)

        # Progress bar
        draw_progress_bar(draw, 300, HEIGHT-80, 200, assembly, f"Assembly: {int(assembly*100)}%")

        # Sparks/work indicators
        if int(frame_num * 3) % 4 < 2:
            spark_x = 450 + int(phase_progress * 100)
            spark_y = 200 + int(math.sin(frame_num) * 30)
            draw.ellipse([spark_x-3, spark_y-3, spark_x+3, spark_y+3], fill=GREEN)

    elif progress < 0.75:
        # Phase 4: Powering on
        phase_progress = (progress - 0.60) / 0.15
        draw.text((WIDTH/2, HEIGHT-40), "Step 4: Activation!", fill=WHITE, anchor="mm")

        # Human stepping back
        draw_human(draw, 150, 280, scale=1.2, celebrating=phase_progress > 0.7)

        # Robot powering on (eyes flickering)
        flicker = phase_progress > 0.3 and (int(frame_num * 2) % 3 != 0 or phase_progress > 0.8)
        draw_robot(draw, 550, 300, scale=1.0, assembly_progress=1.0, powered=flicker, walking_frame=0)

        # Power-on effects
        if phase_progress > 0.5:
            for i in range(3):
                angle = (frame_num * 10 + i * 120) * math.pi / 180
                px = 550 + int(math.cos(angle) * 80)
                py = 250 + int(math.sin(angle) * 80)
                draw.ellipse([px-4, py-4, px+4, py+4], fill=GREEN)

    else:
        # Phase 5: Robot working
        phase_progress = (progress - 0.75) / 0.25
        draw.text((WIDTH/2, HEIGHT-40), "Step 5: Robot Working Like Human!", fill=WHITE, anchor="mm")

        # Human watching proudly
        draw_human(draw, 150, 280, scale=1.2, celebrating=True)

        # Robot walking and waving
        robot_x = 450 + int(math.sin(phase_progress * math.pi * 2) * 50)
        draw_robot(draw, robot_x, 300, scale=1.0, assembly_progress=1.0, powered=True, walking_frame=frame_num)

        # Success indicators
        draw.text((550, 100), "SUCCESS!", fill=GREEN, anchor="mm")

        # Floating tech labels
        labels = ["Autonomous", "Intelligent", "Physical AI"]
        for i, label in enumerate(labels):
            ly = 130 + i * 25 + int(math.sin(frame_num * 0.2 + i) * 5)
            draw.text((550, ly), label, fill=WHITE, anchor="mm")

    # Border
    draw.rectangle([5, 5, WIDTH-5, HEIGHT-5], outline=GREEN, width=2)

    # Frame counter (subtle)
    draw.text((WIDTH-50, HEIGHT-20), f"{frame_num+1}/{total_frames}", fill=DARK_GRAY, anchor="mm")

    return img

def main():
    print("Creating 'Build Robot Journey' GIF...")

    frames = []
    for i in range(FRAMES):
        print(f"  Frame {i+1}/{FRAMES}")
        frame = create_frame(i, FRAMES)
        frames.append(frame)

    # Output path
    output_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(output_dir, "build-robot-journey.gif")

    # Save GIF
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=DURATION,
        loop=0,
        optimize=True
    )

    file_size = os.path.getsize(output_path) / 1024
    print(f"\nGIF saved to: {output_path}")
    print(f"Size: {file_size:.1f} KB")
    print(f"Frames: {FRAMES}")
    print(f"Duration: {FRAMES * DURATION / 1000:.1f} seconds")

if __name__ == "__main__":
    main()
