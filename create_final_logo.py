#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Create a clean, minimalist navbar logo without text
Represents: AI agents, robotics, automation, knowledge, futuristic tech
"""

import base64
import sys
import io
from pathlib import Path
from PIL import Image, ImageDraw

# Fix encoding for Windows
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def create_navbar_logo():
    """Create a clean minimalist logo for navbar placement"""

    # Create transparent image
    img = Image.new('RGBA', (1024, 1024), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    # Color palette - minimal with neon accents
    white = (255, 255, 255, 255)
    black = (20, 20, 30, 255)
    neon_cyan = (0, 255, 200, 255)
    neon_purple = (138, 43, 226, 255)
    neon_green = (57, 255, 20, 255)
    gray = (100, 100, 120, 255)

    center_x, center_y = 512, 512

    # ===== DESIGN 1: Minimalist AI Brain + Robot Arm =====

    # Central AI brain (circuit-like shape)
    # Large outer circle (tech boundary)
    draw.ellipse([350, 350, 674, 674], outline=white, width=8)

    # Inner concentric circles (brain folds)
    draw.ellipse([400, 400, 624, 624], outline=neon_cyan, width=4)
    draw.ellipse([450, 450, 574, 574], outline=neon_purple, width=4)

    # Central core (AI node)
    draw.ellipse([480, 480, 544, 544], fill=neon_cyan)

    # Robot arm extending from side
    # Upper arm (cyan)
    draw.line([(674, 512), (850, 380)], fill=neon_cyan, width=20)
    # Lower arm (purple)
    draw.line([(850, 380), (920, 420)], fill=neon_purple, width=16)
    # Gripper
    draw.ellipse([900, 400, 940, 440], outline=white, width=6)

    # Knowledge nodes surrounding the center
    node_radius = 300
    node_positions = [
        (center_x + node_radius * 0.707, center_y - node_radius * 0.707),  # Top-right
        (center_x + node_radius, center_y),                                  # Right
        (center_x + node_radius * 0.707, center_y + node_radius * 0.707),   # Bottom-right
        (center_x - node_radius * 0.707, center_y + node_radius * 0.707),   # Bottom-left
        (center_x - node_radius, center_y),                                  # Left
        (center_x - node_radius * 0.707, center_y - node_radius * 0.707),   # Top-left
    ]

    for i, (x, y) in enumerate(node_positions):
        # Alternate colors for nodes
        color = neon_green if i % 2 == 0 else neon_purple
        draw.ellipse([x-20, y-20, x+20, y+20], fill=color)
        # Connection line to center
        draw.line([(center_x, center_y), (x, y)], fill=gray, width=2)

    # Automation indicator (rotating arrows around center)
    # Top arrow
    draw.line([(512, 200), (512, 260)], fill=white, width=4)
    draw.polygon([(512, 180), (500, 210), (524, 210)], fill=white)

    # Bottom arrow
    draw.line([(512, 824), (512, 764)], fill=white, width=4)
    draw.polygon([(512, 844), (524, 814), (500, 814)], fill=white)

    # Save PNG
    output_path = Path(__file__).parent / "logo.png"
    img.save(output_path, 'PNG')
    print(f"✓ Logo created: {output_path}")

    # Get file size
    file_size = output_path.stat().st_size
    print(f"✓ File size: {file_size} bytes ({file_size/1024:.1f} KB)")

    # Get base64
    with open(output_path, 'rb') as f:
        png_data = f.read()

    png_b64 = base64.b64encode(png_data).decode()

    # Save base64 to file
    b64_path = Path(__file__).parent / "logo_base64.txt"
    with open(b64_path, 'w') as f:
        f.write("Humanoid-Robotic-Book Logo (Minimalist, No Text)\n")
        f.write("=" * 80 + "\n\n")
        f.write("File: logo.png\n")
        f.write("Size: 1024x1024 pixels\n")
        f.write("Format: PNG with transparent background\n")
        f.write("Design: Minimalist AI brain + robot arm + knowledge nodes\n")
        f.write("No text, no watermarks\n\n")
        f.write(png_b64)

    print(f"✓ Base64 saved: {b64_path}")
    print(f"\n[✓] COMPLETE - Logo ready for navbar/favicon")
    print(f"\nBase64 (first 100 chars): {png_b64[:100]}...")
    print(f"Base64 (last 50 chars): ...{png_b64[-50:]}")
    print(f"\nTotal Base64 size: {len(png_b64)} characters")

if __name__ == "__main__":
    create_navbar_logo()
