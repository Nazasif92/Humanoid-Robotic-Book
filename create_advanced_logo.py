#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Advanced Humanoid-Robotic-Book Logo Generator
Creates a futuristic, professional logo representing AI, robotics, automation, and knowledge
"""

import base64
import sys
from pathlib import Path

# Fix encoding for Windows
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def create_advanced_logo():
    """
    Create an advanced futuristic logo with:
    - Central AI eye (robot eye symbol)
    - Orbital rings (robotics/automation)
    - Knowledge nodes (documentation)
    - Minimalist geometric design
    """

    svg_content = '''<?xml version="1.0" encoding="UTF-8"?>
<svg width="1024" height="1024" viewBox="0 0 1024 1024" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <style>
      /* Define stroke and fill styles */
      .outer-frame { stroke: #E0E0E0; stroke-width: 6; fill: none; }
      .orbital-ring-1 { stroke: #00D4FF; stroke-width: 4; fill: none; opacity: 0.8; }
      .orbital-ring-2 { stroke: #FF00FF; stroke-width: 4; fill: none; opacity: 0.6; }
      .orbital-ring-3 { stroke: #00FF88; stroke-width: 4; fill: none; opacity: 0.4; }

      .ai-eye-outer { fill: #00D4FF; }
      .ai-eye-inner { fill: #FF00FF; }
      .ai-eye-pupil { fill: #FFFFFF; }

      .knowledge-node { fill: #00FF88; }
      .automation-node { fill: #FFD700; }
      .energy-pulse { fill: none; stroke: #00D4FF; stroke-width: 3; }

      .center-core { fill: #FFFFFF; }
      .accent-line { stroke: #E0E0E0; stroke-width: 2; fill: none; }

      /* Animation definitions */
      @keyframes orbit {
        from { transform: rotate(0deg); }
        to { transform: rotate(360deg); }
      }

      .orbiting { animation: orbit 20s linear infinite; transform-origin: center; }
    </style>

    <!-- Gradient definitions -->
    <radialGradient id="aiEyeGradient" cx="50%" cy="50%" r="50%">
      <stop offset="0%" style="stop-color:#00D4FF;stop-opacity:1" />
      <stop offset="100%" style="stop-color:#0080FF;stop-opacity:1" />
    </radialGradient>

    <linearGradient id="techGradient" x1="0%" y1="0%" x2="100%" y2="100%">
      <stop offset="0%" style="stop-color:#00D4FF;stop-opacity:0.8" />
      <stop offset="50%" style="stop-color:#FF00FF;stop-opacity:0.6" />
      <stop offset="100%" style="stop-color:#00FF88;stop-opacity:0.8" />
    </linearGradient>
  </defs>

  <!-- Background (transparent) -->
  <!-- Outer square frame (tech border) -->
  <rect x="50" y="50" width="924" height="924" rx="80" class="outer-frame"/>

  <!-- ORBITAL SYSTEM (Representing Robotics/Automation) -->

  <!-- Outer orbital ring 1 (largest) -->
  <circle cx="512" cy="512" r="380" class="orbital-ring-1"/>

  <!-- Orbital ring 2 (medium) -->
  <circle cx="512" cy="512" r="280" class="orbital-ring-2"/>

  <!-- Orbital ring 3 (inner) -->
  <circle cx="512" cy="512" r="180" class="orbital-ring-3"/>

  <!-- KNOWLEDGE NODES (Documentation/Learning) -->
  <!-- Top node (knowledge) -->
  <circle cx="512" cy="160" r="16" class="knowledge-node"/>
  <circle cx="512" cy="160" r="22" class="orbital-ring-1" stroke-width="1"/>

  <!-- Right node -->
  <circle cx="820" cy="400" r="16" class="knowledge-node"/>
  <circle cx="820" cy="400" r="22" class="orbital-ring-1" stroke-width="1"/>

  <!-- Bottom-right node -->
  <circle cx="750" cy="750" r="16" class="knowledge-node"/>
  <circle cx="750" cy="750" r="22" class="orbital-ring-1" stroke-width="1"/>

  <!-- Bottom-left node -->
  <circle cx="280" cy="750" r="16" class="knowledge-node"/>
  <circle cx="280" cy="750" r="22" class="orbital-ring-1" stroke-width="1"/>

  <!-- Left node -->
  <circle cx="210" cy="400" r="16" class="knowledge-node"/>
  <circle cx="210" cy="400" r="22" class="orbital-ring-1" stroke-width="1"/>

  <!-- AUTOMATION NODES (Technology pulse points) -->
  <!-- Right orbital point -->
  <circle cx="820" cy="512" r="12" class="automation-node"/>

  <!-- Bottom orbital point -->
  <circle cx="512" cy="820" r="12" class="automation-node"/>

  <!-- Left orbital point -->
  <circle cx="204" cy="512" r="12" class="automation-node"/>

  <!-- Top-left diagonal -->
  <circle cx="330" cy="260" r="10" class="automation-node"/>

  <!-- Top-right diagonal -->
  <circle cx="694" cy="260" r="10" class="automation-node"/>

  <!-- CENTRAL AI EYE (Robot Eye Symbol) -->
  <!-- Outer eye circle -->
  <circle cx="512" cy="512" r="90" fill="url(#aiEyeGradient)" opacity="0.9"/>

  <!-- Inner iris circle (purple) -->
  <circle cx="512" cy="512" r="60" class="ai-eye-inner"/>

  <!-- Pupil (white) -->
  <circle cx="512" cy="512" r="32" class="ai-eye-pupil"/>

  <!-- Pupil highlight (creates 3D effect) -->
  <circle cx="520" cy="505" r="12" fill="#FF00FF" opacity="0.6"/>

  <!-- Eye shine (bright reflection) -->
  <circle cx="525" cy="502" r="6" fill="#FFFFFF"/>

  <!-- AI RECOGNITION MARKS (Geometric accents around eye) -->
  <!-- Top accent -->
  <line x1="512" y1="400" x2="512" y2="420" class="accent-line" stroke-width="3"/>
  <!-- Right accent -->
  <line x1="620" y1="512" x2="600" y2="512" class="accent-line" stroke-width="3"/>
  <!-- Bottom accent -->
  <line x1="512" y1="620" x2="512" y2="600" class="accent-line" stroke-width="3"/>
  <!-- Left accent -->
  <line x1="410" y1="512" x2="430" y2="512" class="accent-line" stroke-width="3"/>

  <!-- CONNECTING PATHWAYS (Automation/Data flow) -->
  <!-- Connection from eye to knowledge nodes -->
  <path d="M 512 602 L 512 750" class="accent-line" opacity="0.5"/>
  <path d="M 602 512 L 820 512" class="accent-line" opacity="0.5"/>
  <path d="M 422 512 L 204 512" class="accent-line" opacity="0.5"/>
  <path d="M 512 410 L 512 160" class="accent-line" opacity="0.5"/>

  <!-- DIAGONAL TECH LINES (Representing neural network) -->
  <line x1="380" y1="380" x2="450" y2="450" class="accent-line" opacity="0.3" stroke-width="1.5"/>
  <line x1="650" y1="380" x2="580" y2="450" class="accent-line" opacity="0.3" stroke-width="1.5"/>
  <line x1="380" y1="650" x2="450" y2="580" class="accent-line" opacity="0.3" stroke-width="1.5"/>
  <line x1="650" y1="650" x2="580" y2="580" class="accent-line" opacity="0.3" stroke-width="1.5"/>

  <!-- CORE ELEMENTS (Inner geometric pattern) -->
  <!-- Small corner accents (representing robotics precision) -->
  <circle cx="150" cy="150" r="8" fill="#E0E0E0" opacity="0.6"/>
  <circle cx="874" cy="150" r="8" fill="#E0E0E0" opacity="0.6"/>
  <circle cx="150" cy="874" r="8" fill="#E0E0E0" opacity="0.6"/>
  <circle cx="874" cy="874" r="8" fill="#E0E0E0" opacity="0.6"/>

  <!-- TECHNOLOGY PULSE INDICATORS -->
  <!-- Pulsing circles around orbital ring 2 -->
  <circle cx="512" cy="232" r="6" class="automation-node" opacity="0.8"/>
  <circle cx="752" cy="512" r="6" class="automation-node" opacity="0.8"/>
  <circle cx="512" cy="792" r="6" class="automation-node" opacity="0.8"/>
  <circle cx="272" cy="512" r="6" class="automation-node" opacity="0.8"/>

  <!-- FUTURISTIC GLOW EFFECT (subtle) -->
  <defs>
    <filter id="glow">
      <feGaussianBlur stdDeviation="3" result="coloredBlur"/>
      <feMerge>
        <feMergeNode in="coloredBlur"/>
        <feMergeNode in="SourceGraphic"/>
      </feMerge>
    </filter>
  </defs>

  <!-- Apply glow to central eye -->
  <circle cx="512" cy="512" r="95" fill="none" stroke="#00D4FF" stroke-width="2" opacity="0.3" filter="url(#glow)"/>

</svg>'''

    # Save SVG to file
    svg_path = Path(__file__).parent / "logo.svg"
    with open(svg_path, 'w') as f:
        f.write(svg_content)
    print("[+] SVG logo saved: logo.svg")

    # Convert to base64 for documentation
    svg_b64 = base64.b64encode(svg_content.encode()).decode()
    print("[+] SVG Base64 encoding generated")

    # Create PNG using Pillow
    try:
        from PIL import Image, ImageDraw

        # Create a new RGBA image with transparent background
        img = Image.new('RGBA', (1024, 1024), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)

        # Color definitions
        light_gray = (224, 224, 224, 255)
        cyan = (0, 212, 255, 255)
        magenta = (255, 0, 255, 255)
        lime = (0, 255, 136, 255)
        gold = (255, 215, 0, 255)
        white = (255, 255, 255, 255)
        cyan_light = (0, 128, 255, 255)

        # Draw outer square frame
        margin = 50
        frame_radius = 80
        draw.rectangle([margin, margin, 1024-margin, 1024-margin],
                      outline=light_gray, width=6)

        # Draw orbital rings
        # Ring 1 (cyan)
        draw.ellipse([512-380, 512-380, 512+380, 512+380],
                    outline=cyan, width=4)

        # Ring 2 (magenta)
        draw.ellipse([512-280, 512-280, 512+280, 512+280],
                    outline=magenta, width=4)

        # Ring 3 (lime)
        draw.ellipse([512-180, 512-180, 512+180, 512+180],
                    outline=lime, width=4)

        # Draw knowledge nodes (cyan circles)
        knowledge_positions = [
            (512, 160),   # Top
            (820, 400),   # Right
            (750, 750),   # Bottom-right
            (280, 750),   # Bottom-left
            (210, 400),   # Left
        ]

        for x, y in knowledge_positions:
            draw.ellipse([x-16, y-16, x+16, y+16], fill=lime)
            draw.ellipse([x-22, y-22, x+22, y+22], outline=cyan, width=1)

        # Draw automation nodes (gold circles)
        automation_positions = [
            (820, 512), (512, 820), (204, 512),
            (330, 260), (694, 260),
        ]

        for x, y in automation_positions:
            size = 12 if (x, y) in [(820, 512), (512, 820), (204, 512)] else 10
            draw.ellipse([x-size, y-size, x+size, y+size], fill=gold)

        # Draw central AI eye
        # Outer eye circle (cyan)
        draw.ellipse([512-90, 512-90, 512+90, 512+90], fill=cyan)

        # Inner iris (magenta)
        draw.ellipse([512-60, 512-60, 512+60, 512+60], fill=magenta)

        # Pupil (white)
        draw.ellipse([512-32, 512-32, 512+32, 512+32], fill=white)

        # Pupil highlight
        draw.ellipse([520-12, 505-12, 520+12, 505+12], fill=magenta)

        # Eye shine
        draw.ellipse([525-6, 502-6, 525+6, 502+6], fill=white)

        # Draw AI recognition marks
        draw.line([(512, 400), (512, 420)], fill=light_gray, width=3)  # Top
        draw.line([(620, 512), (600, 512)], fill=light_gray, width=3)  # Right
        draw.line([(512, 620), (512, 600)], fill=light_gray, width=3)  # Bottom
        draw.line([(410, 512), (430, 512)], fill=light_gray, width=3)  # Left

        # Draw connecting pathways
        draw.line([(512, 602), (512, 750)], fill=light_gray, width=1)
        draw.line([(602, 512), (820, 512)], fill=light_gray, width=1)
        draw.line([(422, 512), (204, 512)], fill=light_gray, width=1)
        draw.line([(512, 410), (512, 160)], fill=light_gray, width=1)

        # Draw diagonal tech lines
        draw.line([(380, 380), (450, 450)], fill=light_gray, width=1)
        draw.line([(650, 380), (580, 450)], fill=light_gray, width=1)
        draw.line([(380, 650), (450, 580)], fill=light_gray, width=1)
        draw.line([(650, 650), (580, 580)], fill=light_gray, width=1)

        # Draw corner accents
        corners = [(150, 150), (874, 150), (150, 874), (874, 874)]
        for x, y in corners:
            draw.ellipse([x-8, y-8, x+8, y+8], fill=light_gray)

        # Draw technology pulse indicators
        pulse_positions = [(512, 232), (752, 512), (512, 792), (272, 512)]
        for x, y in pulse_positions:
            draw.ellipse([x-6, y-6, x+6, y+6], fill=gold)

        # Save PNG
        output_path = Path(__file__).parent / "logo.png"
        img.save(output_path, 'PNG')
        print("[+] PNG logo created: logo.png (14 KB)")

        # Get base64 encoding
        with open(output_path, 'rb') as f:
            png_data = f.read()
            png_b64 = base64.b64encode(png_data).decode()

        # Save base64 to file
        b64_path = Path(__file__).parent / "logo-base64.txt"
        with open(b64_path, 'w') as f:
            f.write("Advanced Humanoid-Robotic-Book Logo (PNG Base64)\n")
            f.write("=" * 80 + "\n\n")
            f.write("File: logo.png\n")
            f.write("Size: 1024x1024 pixels\n")
            f.write("Format: PNG with transparent background\n")
            f.write("Design: AI Robot Eye with Orbital Rings\n\n")
            f.write(png_b64)

        print("[+] Base64 encoding saved: logo-base64.txt")
        print(f"\n[SUCCESS] Logo generation complete!")
        print(f"PNG File Size: {len(png_data)} bytes")
        print(f"Base64 Size: {len(png_b64)} characters")
        print(f"\nBase64 (first 100 chars): {png_b64[:100]}...")
        print(f"Base64 (last 50 chars): ...{png_b64[-50:]}")

    except ImportError as e:
        print(f"[WARNING] Pillow not available: {e}")
        print("[INFO] SVG logo generated successfully")
        print("[INFO] Use logo.svg for web deployment (recommended)")

if __name__ == "__main__":
    create_advanced_logo()
