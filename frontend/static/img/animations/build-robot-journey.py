#!/usr/bin/env python3
"""
Educational Journey GIF: From Learning to Creation
Based on research from:
- Educational Voice animated infographics guidelines
- Graphics for Learning (Ruth Clark & Chopeta Lyons)
- UAF Center for Teaching and Learning GIF guidelines

Best Practices Applied:
- 6 clear stages (10-20 seconds total at ~2.5 sec/stage)
- Visual hierarchy with highlighted active elements
- Color coding: Green (success), Blue (learning), Orange (building)
- Progress indicators for each stage
- Clear text labels with step numbers
- Smooth transitions between stages
"""

from PIL import Image, ImageDraw, ImageFont
import math
import os

# Configuration - Educational GIF best practices
WIDTH, HEIGHT = 900, 550  # Wider for better storytelling
FRAMES = 72  # 6 stages x 12 frames = ~15 seconds at 5 FPS
FPS = 5  # Slower for comprehension
DURATION = int(1000 / FPS)

# Color Scheme (Research-based educational palette)
BG_COLOR = (20, 25, 30)       # Dark slate background
PRIMARY_GREEN = (118, 185, 0)  # NVIDIA green - success/completion
SECONDARY_BLUE = (66, 133, 244)  # Google blue - learning/info
ACCENT_ORANGE = (255, 152, 0)   # Orange - building/progress
WHITE = (255, 255, 255)
LIGHT_GRAY = (200, 200, 200)
MEDIUM_GRAY = (120, 120, 120)
DARK_GRAY = (60, 60, 60)
HIGHLIGHT = (255, 235, 59)  # Yellow highlight for active elements

# Stage definitions for clear storytelling
STAGES = [
    {"name": "DISCOVER", "subtitle": "Open the Physical AI Book", "color": SECONDARY_BLUE, "icon": "book"},
    {"name": "LEARN", "subtitle": "Master ROS2, Gazebo, Isaac & VLA", "color": SECONDARY_BLUE, "icon": "brain"},
    {"name": "GATHER", "subtitle": "Collect Robot Components", "color": ACCENT_ORANGE, "icon": "parts"},
    {"name": "BUILD", "subtitle": "Assemble Your Humanoid Robot", "color": ACCENT_ORANGE, "icon": "tools"},
    {"name": "ACTIVATE", "subtitle": "Power On & Initialize AI", "color": PRIMARY_GREEN, "icon": "power"},
    {"name": "ACHIEVE", "subtitle": "Your Robot Works Autonomously!", "color": PRIMARY_GREEN, "icon": "success"},
]


def draw_rounded_rect(draw, coords, fill, outline=None, radius=10, width=2):
    """Draw a rounded rectangle"""
    x1, y1, x2, y2 = coords
    # Simple approximation with regular rectangle for PIL compatibility
    draw.rectangle(coords, fill=fill, outline=outline, width=width)


def draw_stage_indicator(draw, current_stage, total_stages=6):
    """Draw progress dots at the bottom"""
    dot_spacing = 50
    start_x = WIDTH // 2 - (total_stages - 1) * dot_spacing // 2
    y = HEIGHT - 45

    for i in range(total_stages):
        x = start_x + i * dot_spacing
        if i < current_stage:
            # Completed
            draw.ellipse([x-8, y-8, x+8, y+8], fill=PRIMARY_GREEN)
            draw.text((x, y), "✓", fill=WHITE, anchor="mm")
        elif i == current_stage:
            # Current (highlighted)
            draw.ellipse([x-10, y-10, x+10, y+10], fill=HIGHLIGHT, outline=WHITE, width=2)
            draw.text((x, y), str(i+1), fill=BG_COLOR, anchor="mm")
        else:
            # Upcoming
            draw.ellipse([x-8, y-8, x+8, y+8], fill=DARK_GRAY, outline=MEDIUM_GRAY, width=1)
            draw.text((x, y), str(i+1), fill=MEDIUM_GRAY, anchor="mm")


def draw_header(draw, stage_num, stage_info, frame_in_stage):
    """Draw the header with stage info"""
    # Stage number badge
    badge_x, badge_y = 60, 45
    draw.ellipse([badge_x-25, badge_y-25, badge_x+25, badge_y+25],
                 fill=stage_info["color"], outline=WHITE, width=2)
    draw.text((badge_x, badge_y), str(stage_num + 1), fill=WHITE, anchor="mm")

    # Stage name with animation
    pulse = math.sin(frame_in_stage * 0.5) * 0.1 + 1.0
    draw.text((130, 35), stage_info["name"], fill=stage_info["color"], anchor="lm")
    draw.text((130, 60), stage_info["subtitle"], fill=LIGHT_GRAY, anchor="lm")

    # Decorative line
    draw.line([130, 80, 130 + len(stage_info["subtitle"]) * 7, 80],
              fill=stage_info["color"], width=2)


def draw_human_student(draw, x, y, scale=1.0, pose="standing", highlight=False):
    """Draw a friendly human student figure"""
    s = scale
    outline_color = HIGHLIGHT if highlight else WHITE
    outline_width = 3 if highlight else 2

    # Head (circle)
    draw.ellipse([x-18*s, y-85*s, x+18*s, y-50*s],
                 fill=LIGHT_GRAY, outline=outline_color, width=outline_width)

    # Simple face
    # Eyes
    draw.ellipse([x-10*s, y-75*s, x-4*s, y-68*s], fill=BG_COLOR)
    draw.ellipse([x+4*s, y-75*s, x+10*s, y-68*s], fill=BG_COLOR)
    # Smile
    draw.arc([x-8*s, y-70*s, x+8*s, y-58*s], 0, 180, fill=BG_COLOR, width=2)

    # Body
    draw.rectangle([x-22*s, y-50*s, x+22*s, y+15*s],
                   fill=SECONDARY_BLUE, outline=outline_color, width=outline_width)

    if pose == "reading":
        # Arms in front (holding book)
        draw.rectangle([x-40*s, y-35*s, x-22*s, y-5*s], fill=LIGHT_GRAY, outline=outline_color, width=2)
        draw.rectangle([x+22*s, y-35*s, x+40*s, y-5*s], fill=LIGHT_GRAY, outline=outline_color, width=2)
    elif pose == "thinking":
        # Hand on chin
        draw.rectangle([x-40*s, y-45*s, x-22*s, y-20*s], fill=LIGHT_GRAY, outline=outline_color, width=2)
        draw.rectangle([x+22*s, y-35*s, x+35*s, y+10*s], fill=LIGHT_GRAY, outline=outline_color, width=2)
    elif pose == "building":
        # Arms extended to work
        draw.rectangle([x+22*s, y-40*s, x+65*s, y-25*s], fill=LIGHT_GRAY, outline=outline_color, width=2)
        draw.rectangle([x+22*s, y-15*s, x+55*s, y+0*s], fill=LIGHT_GRAY, outline=outline_color, width=2)
    elif pose == "celebrating":
        # Arms raised
        draw.rectangle([x-45*s, y-70*s, x-22*s, y-40*s], fill=LIGHT_GRAY, outline=outline_color, width=2)
        draw.rectangle([x+22*s, y-70*s, x+45*s, y-40*s], fill=LIGHT_GRAY, outline=outline_color, width=2)
    else:
        # Arms at sides
        draw.rectangle([x-40*s, y-45*s, x-22*s, y+10*s], fill=LIGHT_GRAY, outline=outline_color, width=2)
        draw.rectangle([x+22*s, y-45*s, x+40*s, y+10*s], fill=LIGHT_GRAY, outline=outline_color, width=2)

    # Legs
    draw.rectangle([x-18*s, y+15*s, x-5*s, y+60*s], fill=DARK_GRAY, outline=outline_color, width=2)
    draw.rectangle([x+5*s, y+15*s, x+18*s, y+60*s], fill=DARK_GRAY, outline=outline_color, width=2)


def draw_book(draw, x, y, scale=1.0, open_progress=0, highlight=False):
    """Draw the Physical AI textbook"""
    s = scale
    outline_color = HIGHLIGHT if highlight else PRIMARY_GREEN

    w, h = 70*s, 90*s

    # Book cover
    draw.rectangle([x-w/2, y-h/2, x+w/2, y+h/2],
                   fill=(30, 60, 30), outline=outline_color, width=3)

    # Title area
    draw.rectangle([x-w/2+8, y-h/2+10, x+w/2-8, y-h/2+40], fill=PRIMARY_GREEN)
    draw.text((x, y-h/2+25), "PHYSICAL", fill=WHITE, anchor="mm")
    draw.text((x, y-h/2+50), "AI", fill=PRIMARY_GREEN, anchor="mm")

    # Spine
    draw.line([x-w/2+3, y-h/2+3, x-w/2+3, y+h/2-3], fill=outline_color, width=2)

    # Pages visible if open
    if open_progress > 0:
        page_width = int(15 * open_progress * s)
        draw.rectangle([x+w/2, y-h/2+5, x+w/2+page_width, y+h/2-5], fill=WHITE)


def draw_knowledge_bubbles(draw, x, y, progress, topics):
    """Draw floating knowledge topic bubbles"""
    visible = int(progress * len(topics))

    for i, topic in enumerate(topics[:visible]):
        angle = (i / len(topics)) * math.pi - math.pi/2
        radius = 100 + i * 20
        bx = x + int(math.cos(angle + progress * 0.5) * radius)
        by = y + int(math.sin(angle + progress * 0.5) * radius * 0.6)

        # Bubble
        text_width = len(topic) * 8 + 20
        draw.ellipse([bx-text_width/2, by-15, bx+text_width/2, by+15],
                     fill=DARK_GRAY, outline=SECONDARY_BLUE, width=2)
        draw.text((bx, by), topic, fill=WHITE, anchor="mm")


def draw_robot_parts(draw, x, y, visible_parts, highlight_part=-1):
    """Draw scattered robot parts"""
    parts = [
        {"name": "HEAD", "offset": (-80, -60), "size": (50, 45), "color": DARK_GRAY},
        {"name": "TORSO", "offset": (0, 20), "size": (65, 70), "color": DARK_GRAY},
        {"name": "ARM L", "offset": (-100, 30), "size": (20, 55), "color": DARK_GRAY},
        {"name": "ARM R", "offset": (100, 30), "size": (20, 55), "color": DARK_GRAY},
        {"name": "LEG L", "offset": (-50, 90), "size": (22, 60), "color": DARK_GRAY},
        {"name": "LEG R", "offset": (50, 90), "size": (22, 60), "color": DARK_GRAY},
    ]

    for i, part in enumerate(parts[:visible_parts]):
        px = x + part["offset"][0]
        py = y + part["offset"][1]
        w, h = part["size"]

        outline = HIGHLIGHT if i == highlight_part else ACCENT_ORANGE
        draw.rectangle([px-w/2, py-h/2, px+w/2, py+h/2],
                       fill=part["color"], outline=outline, width=3 if i == highlight_part else 2)
        draw.text((px, py), part["name"], fill=outline, anchor="mm")


def draw_robot(draw, x, y, scale=1.0, assembly=1.0, powered=False, anim_frame=0):
    """Draw the humanoid robot at various assembly stages"""
    s = scale

    glow_color = PRIMARY_GREEN if powered else DARK_GRAY
    outline = PRIMARY_GREEN if powered else ACCENT_ORANGE

    # Only draw parts based on assembly progress
    if assembly < 0.15:
        return

    # Legs (first)
    if assembly >= 0.15:
        leg_offset = int(math.sin(anim_frame * 0.4) * 8) if powered else 0
        # Left leg
        draw.rectangle([x-28*s, y+25*s, x-8*s, y+80*s+leg_offset], fill=BG_COLOR, outline=outline, width=2)
        draw.rectangle([x-32*s, y+75*s+leg_offset, x-5*s, y+90*s+leg_offset], fill=DARK_GRAY, outline=outline, width=2)
        # Right leg
        draw.rectangle([x+8*s, y+25*s, x+28*s, y+80*s-leg_offset], fill=BG_COLOR, outline=outline, width=2)
        draw.rectangle([x+5*s, y+75*s-leg_offset, x+32*s, y+90*s-leg_offset], fill=DARK_GRAY, outline=outline, width=2)

    # Torso
    if assembly >= 0.35:
        draw.rectangle([x-35*s, y-30*s, x+35*s, y+30*s], fill=BG_COLOR, outline=outline, width=2)
        # Chest panel
        draw.rectangle([x-25*s, y-20*s, x+25*s, y+15*s], fill=DARK_GRAY, outline=glow_color, width=1)

        if powered:
            # Glowing indicators
            for i, cx in enumerate([-15, 0, 15]):
                color = [PRIMARY_GREEN, SECONDARY_BLUE, ACCENT_ORANGE][i]
                pulse = int(math.sin(anim_frame * 0.5 + i) * 2) + 5
                draw.ellipse([x+cx*s-pulse, y-5*s-pulse, x+cx*s+pulse, y-5*s+pulse], fill=color)

    # Arms
    if assembly >= 0.55:
        wave = int(math.sin(anim_frame * 0.3) * 10) if powered else 0

        # Left arm
        draw.rectangle([x-55*s, y-25*s, x-35*s, y+20*s], fill=BG_COLOR, outline=outline, width=2)
        draw.rectangle([x-60*s, y+15*s-wave, x-40*s, y+50*s-wave], fill=BG_COLOR, outline=outline, width=2)

        # Right arm (waving if powered)
        if powered and anim_frame % 24 > 12:
            draw.rectangle([x+35*s, y-25*s, x+55*s, y+5*s], fill=BG_COLOR, outline=outline, width=2)
            draw.rectangle([x+40*s, y-60*s, x+60*s, y-20*s], fill=BG_COLOR, outline=outline, width=2)
            # Hand
            draw.ellipse([x+42*s, y-72*s, x+58*s, y-58*s], fill=DARK_GRAY, outline=outline, width=2)
        else:
            draw.rectangle([x+35*s, y-25*s, x+55*s, y+20*s], fill=BG_COLOR, outline=outline, width=2)
            draw.rectangle([x+40*s, y+15*s+wave, x+60*s, y+50*s+wave], fill=BG_COLOR, outline=outline, width=2)

    # Head
    if assembly >= 0.75:
        # Neck
        draw.rectangle([x-12*s, y-42*s, x+12*s, y-28*s], fill=DARK_GRAY, outline=glow_color, width=1)
        # Head
        draw.rectangle([x-30*s, y-90*s, x+30*s, y-40*s], fill=BG_COLOR, outline=outline, width=2)
        # Visor
        draw.rectangle([x-22*s, y-82*s, x+22*s, y-55*s], fill=(20, 30, 40), outline=glow_color, width=1)

        # Eyes
        if powered:
            blink = anim_frame % 36 < 3
            eye_h = 3 if blink else 8
            draw.ellipse([x-14*s-6, y-70*s-eye_h, x-14*s+6, y-70*s+eye_h], fill=PRIMARY_GREEN)
            draw.ellipse([x+14*s-6, y-70*s-eye_h, x+14*s+6, y-70*s+eye_h], fill=PRIMARY_GREEN)
            # Eye shine
            draw.ellipse([x-14*s-2, y-70*s-2, x-14*s+2, y-70*s+2], fill=WHITE)
            draw.ellipse([x+14*s-2, y-70*s-2, x+14*s+2, y-70*s+2], fill=WHITE)
        else:
            draw.ellipse([x-14*s-6, y-70*s-6, x-14*s+6, y-70*s+6], fill=DARK_GRAY)
            draw.ellipse([x+14*s-6, y-70*s-6, x+14*s+6, y-70*s+6], fill=DARK_GRAY)

        # Antenna
        draw.line([x, y-90*s, x, y-108*s], fill=outline, width=3)
        if powered:
            pulse = int(math.sin(anim_frame * 0.5) * 4) + 8
            draw.ellipse([x-pulse, y-115*s-pulse, x+pulse, y-115*s+pulse], fill=PRIMARY_GREEN)
        else:
            draw.ellipse([x-6, y-115*s-6, x+6, y-115*s+6], fill=DARK_GRAY)


def draw_progress_bar(draw, x, y, width, progress, label, color):
    """Draw an animated progress bar"""
    height = 25

    # Background
    draw.rectangle([x, y, x+width, y+height], fill=DARK_GRAY, outline=MEDIUM_GRAY, width=1)

    # Fill
    fill_width = int((width - 4) * progress)
    if fill_width > 0:
        draw.rectangle([x+2, y+2, x+2+fill_width, y+height-2], fill=color)

    # Label and percentage
    draw.text((x + width/2, y + height + 18), label, fill=WHITE, anchor="mm")
    draw.text((x + width + 15, y + height/2), f"{int(progress*100)}%", fill=color, anchor="lm")


def draw_success_effects(draw, x, y, frame):
    """Draw celebratory effects"""
    # Radiating lines
    for i in range(8):
        angle = (i / 8) * math.pi * 2 + frame * 0.1
        length = 60 + math.sin(frame * 0.5 + i) * 20
        ex = x + int(math.cos(angle) * length)
        ey = y + int(math.sin(angle) * length)
        draw.line([x, y, ex, ey], fill=PRIMARY_GREEN, width=2)

    # Stars
    for i in range(5):
        angle = (i / 5) * math.pi * 2 + frame * 0.15
        dist = 100 + i * 15
        sx = x + int(math.cos(angle) * dist)
        sy = y + int(math.sin(angle) * dist)
        size = 6 + int(math.sin(frame * 0.3 + i) * 3)
        draw.polygon([
            (sx, sy-size), (sx+size//2, sy-size//3), (sx+size, sy),
            (sx+size//2, sy+size//3), (sx, sy+size),
            (sx-size//2, sy+size//3), (sx-size, sy), (sx-size//2, sy-size//3)
        ], fill=HIGHLIGHT, outline=WHITE)


def create_frame(frame_num, total_frames):
    """Create a single frame with proper educational staging"""
    img = Image.new('RGB', (WIDTH, HEIGHT), BG_COLOR)
    draw = ImageDraw.Draw(img)

    # Calculate which stage and progress within stage
    frames_per_stage = total_frames // len(STAGES)
    current_stage = min(frame_num // frames_per_stage, len(STAGES) - 1)
    frame_in_stage = frame_num % frames_per_stage
    stage_progress = frame_in_stage / frames_per_stage

    stage_info = STAGES[current_stage]

    # Draw header
    draw_header(draw, current_stage, stage_info, frame_in_stage)

    # Draw stage indicators
    draw_stage_indicator(draw, current_stage, len(STAGES))

    # Main content area
    content_y = 280

    # Stage-specific content
    if current_stage == 0:  # DISCOVER
        # Human finds the book
        human_x = 200 + int(stage_progress * 50)
        draw_human_student(draw, human_x, content_y, scale=1.0, pose="standing", highlight=True)
        draw_book(draw, 400, content_y - 20, scale=1.2, open_progress=0, highlight=stage_progress > 0.5)

        # Arrow pointing to book
        if stage_progress > 0.3:
            draw.polygon([(320, content_y-30), (350, content_y-50), (350, content_y-10)],
                        fill=SECONDARY_BLUE)

    elif current_stage == 1:  # LEARN
        # Human reading and knowledge appearing
        draw_human_student(draw, 200, content_y, scale=1.0, pose="reading")
        draw_book(draw, 200, content_y - 60, scale=1.0, open_progress=stage_progress)
        draw_knowledge_bubbles(draw, 450, content_y - 30, stage_progress,
                              ["ROS2", "Gazebo", "Isaac Sim", "VLA Models"])

        # Progress bar
        draw_progress_bar(draw, 550, content_y + 60, 200, stage_progress, "Knowledge", SECONDARY_BLUE)

    elif current_stage == 2:  # GATHER
        # Human collecting parts
        draw_human_student(draw, 180, content_y, scale=1.0, pose="standing", highlight=stage_progress < 0.3)
        visible = int(stage_progress * 6) + 1
        highlight = int(stage_progress * 6) % 6
        draw_robot_parts(draw, 550, content_y, visible, highlight)

        # Arrow from human to parts
        draw.line([250, content_y, 450, content_y], fill=ACCENT_ORANGE, width=2)
        draw.polygon([(440, content_y-10), (460, content_y), (440, content_y+10)], fill=ACCENT_ORANGE)

    elif current_stage == 3:  # BUILD
        # Human building robot
        draw_human_student(draw, 200, content_y, scale=1.0, pose="building")
        assembly = min(1.0, stage_progress * 1.2)
        draw_robot(draw, 550, content_y + 20, scale=1.0, assembly=assembly, powered=False, anim_frame=frame_num)

        # Progress bar
        draw_progress_bar(draw, 350, content_y + 120, 250, assembly, "Assembly Progress", ACCENT_ORANGE)

        # Sparks during building
        if stage_progress < 0.9:
            spark_x = 480 + int(stage_progress * 80)
            spark_y = content_y - 50 + int(math.sin(frame_num * 0.5) * 30)
            for i in range(3):
                sx = spark_x + int(math.cos(frame_num + i * 2) * 10)
                sy = spark_y + int(math.sin(frame_num + i * 2) * 10)
                draw.ellipse([sx-3, sy-3, sx+3, sy+3], fill=HIGHLIGHT)

    elif current_stage == 4:  # ACTIVATE
        # Robot powering on
        draw_human_student(draw, 200, content_y, scale=1.0, pose="standing")

        # Flicker effect
        powered = stage_progress > 0.4 and (int(frame_num * 3) % 5 != 0 or stage_progress > 0.8)
        draw_robot(draw, 550, content_y + 20, scale=1.0, assembly=1.0, powered=powered, anim_frame=frame_num)

        # Power-on ring effect
        if stage_progress > 0.3:
            ring_size = int((stage_progress - 0.3) * 150)
            alpha = int(255 * (1 - stage_progress))
            draw.ellipse([550-ring_size, content_y+20-ring_size, 550+ring_size, content_y+20+ring_size],
                        outline=PRIMARY_GREEN, width=3)

        # Progress bar
        draw_progress_bar(draw, 350, content_y + 120, 250, stage_progress, "Initialization", PRIMARY_GREEN)

    elif current_stage == 5:  # ACHIEVE
        # Success! Robot working
        draw_human_student(draw, 200, content_y, scale=1.0, pose="celebrating", highlight=True)

        robot_x = 550 + int(math.sin(stage_progress * math.pi * 2) * 30)
        draw_robot(draw, robot_x, content_y + 20, scale=1.0, assembly=1.0, powered=True, anim_frame=frame_num)

        # Success effects
        draw_success_effects(draw, 550, content_y - 50, frame_num)

        # Success text
        draw.text((WIDTH/2, content_y + 140), "YOUR ROBOT IS READY!", fill=PRIMARY_GREEN, anchor="mm")

        # Capability labels
        capabilities = ["Autonomous", "Intelligent", "Physical AI"]
        for i, cap in enumerate(capabilities):
            cy = 100 + i * 30 + int(math.sin(frame_num * 0.2 + i) * 3)
            draw.text((750, cy), f"✓ {cap}", fill=WHITE, anchor="lm")

    # Border
    draw.rectangle([3, 3, WIDTH-3, HEIGHT-3], outline=stage_info["color"], width=2)

    return img


def main():
    print("Creating Enhanced Educational Journey GIF...")
    print(f"Based on research from Educational Voice, Graphics for Learning")
    print(f"Stages: {len(STAGES)}, Frames: {FRAMES}, Duration: ~{FRAMES/FPS:.1f} seconds")

    frames = []
    for i in range(FRAMES):
        stage = i // (FRAMES // len(STAGES))
        print(f"  Frame {i+1}/{FRAMES} - Stage: {STAGES[min(stage, len(STAGES)-1)]['name']}")
        frame = create_frame(i, FRAMES)
        frames.append(frame)

    # Output path
    output_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(output_dir, "build-robot-journey.gif")

    # Save GIF with optimized settings
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=DURATION,
        loop=0,
        optimize=True
    )

    file_size = os.path.getsize(output_path) / 1024
    print(f"\n✓ GIF saved to: {output_path}")
    print(f"  Size: {file_size:.1f} KB")
    print(f"  Frames: {FRAMES}")
    print(f"  Duration: {FRAMES * DURATION / 1000:.1f} seconds")
    print(f"  FPS: {FPS}")


if __name__ == "__main__":
    main()
