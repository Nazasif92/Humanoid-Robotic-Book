#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Humanoid-Robotic-Book Logo Generator
Converts SVG logo to PNG with transparent background
"""

import base64
import struct
import zlib
import sys
from pathlib import Path

# Fix encoding for Windows
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def create_png_from_svg_data():
    """
    Create a PNG version of the logo by rendering SVG as PNG.
    Using a pure Python PNG writer approach.
    """

    # SVG content (same as in generate-logo.js)
    svg_content = '''<?xml version="1.0" encoding="UTF-8"?>
<svg width="1024" height="1024" viewBox="0 0 1024 1024" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <style>
      .tech-circle { stroke: #00FFC8; stroke-width: 8; fill: none; }
      .knowledge-node { fill: #00FFC8; }
      .knowledge-node-large { fill: #C800FF; }
      .robot-arm-v { stroke: #00FFC8; stroke-width: 20; stroke-linecap: round; fill: none; }
      .robot-arm-h { stroke: #C800FF; stroke-width: 20; stroke-linecap: round; fill: none; }
      .joint { fill: white; }
      .agent-node { fill: #00FFC8; }
      .agent-node-alt { fill: #C800FF; }
      .agent-line { stroke: rgba(0, 255, 200, 0.6); stroke-width: 3; fill: none; }
      .knowledge-line { stroke: rgba(200, 0, 255, 0.4); stroke-width: 2; fill: none; }
    </style>
  </defs>
  <circle cx="512" cy="512" r="490" class="tech-circle"/>
  <line x1="300" y1="200" x2="450" y2="280" class="knowledge-line"/>
  <line x1="450" y1="280" x2="600" y2="200" class="knowledge-line"/>
  <line x1="450" y1="280" x2="512" y2="450" class="knowledge-line"/>
  <circle cx="300" cy="200" r="24" class="knowledge-node"/>
  <circle cx="600" cy="200" r="24" class="knowledge-node"/>
  <circle cx="512" cy="320" r="32" class="knowledge-node-large"/>
  <line x1="512" y1="400" x2="512" y2="620" class="robot-arm-v"/>
  <line x1="512" y1="620" x2="720" y2="620" class="robot-arm-h"/>
  <circle cx="512" cy="620" r="18" class="joint"/>
  <circle cx="720" cy="620" r="14" class="joint" fill="#00FFC8" opacity="0.8"/>
  <line x1="350" y1="800" x2="450" y2="750" class="agent-line"/>
  <line x1="450" y1="750" x2="512" y2="760" class="agent-line"/>
  <line x1="512" y1="760" x2="574" y2="750" class="agent-line"/>
  <line x1="574" y1="750" x2="674" y2="800" class="agent-line"/>
  <line x1="350" y1="800" x2="674" y2="800" class="agent-line"/>
  <circle cx="350" cy="800" r="20" class="agent-node"/>
  <circle cx="450" cy="750" r="20" class="agent-node-alt"/>
  <circle cx="512" cy="760" r="22" class="agent-node"/>
  <circle cx="574" cy="750" r="20" class="agent-node-alt"/>
  <circle cx="674" cy="800" r="20" class="agent-node"/>
</svg>'''

    # Save SVG to file
    svg_path = Path(__file__).parent / "logo.svg"
    with open(svg_path, 'w') as f:
        f.write(svg_content)
    print(f"✓ Saved SVG logo: {svg_path}")

    # Convert to base64 for documentation
    svg_b64 = base64.b64encode(svg_content.encode()).decode()
    print(f"\n✓ SVG Base64 (first 100 chars): {svg_b64[:100]}...")

    # Create PNG wrapper (minimal valid PNG)
    # Using cairosvg or rsvg is needed for proper rendering
    # But we'll create a reference PNG with proper structure

    try:
        # Try using cairosvg if available
        try:
            import cairosvg
            output_path = Path(__file__).parent / "logo.png"
            cairosvg.svg2png(bytestring=svg_content.encode(), write_to=str(output_path))
            print(f"✓ Created PNG using cairosvg: {output_path}")

            with open(output_path, 'rb') as f:
                png_data = f.read()
                png_b64 = base64.b64encode(png_data).decode()
                print(f"✓ PNG Base64 (first 100 chars): {png_b64[:100]}...")
                return
        except ImportError:
            pass

        # Try using PIL/Pillow with a workaround
        try:
            from PIL import Image, ImageDraw

            # Create a new RGBA image with transparent background
            img = Image.new('RGBA', (1024, 1024), (0, 0, 0, 0))
            draw = ImageDraw.Draw(img)

            # Colors
            cyan = (0, 255, 200, 255)
            purple = (200, 0, 255, 255)
            white = (255, 255, 255, 255)

            # Draw tech circle
            draw.ellipse([22, 22, 1002, 1002], outline=cyan, width=8)

            # Draw knowledge section
            draw.line([(300, 200), (450, 280)], fill=purple, width=2)
            draw.line([(450, 280), (600, 200)], fill=purple, width=2)
            draw.line([(450, 280), (512, 450)], fill=purple, width=2)
            draw.ellipse([276, 176, 324, 224], fill=cyan)
            draw.ellipse([576, 176, 624, 224], fill=cyan)
            draw.ellipse([480, 288, 544, 352], fill=purple)

            # Draw robot arm
            draw.line([(512, 400), (512, 620)], fill=cyan, width=20)
            draw.line([(512, 620), (720, 620)], fill=purple, width=20)
            draw.ellipse([494, 602, 530, 638], fill=white)
            draw.ellipse([706, 606, 734, 634], fill=cyan)

            # Draw agent network
            draw.line([(350, 800), (450, 750)], fill=cyan, width=3)
            draw.line([(450, 750), (512, 760)], fill=cyan, width=3)
            draw.line([(512, 760), (574, 750)], fill=cyan, width=3)
            draw.line([(574, 750), (674, 800)], fill=cyan, width=3)
            draw.line([(350, 800), (674, 800)], fill=cyan, width=3)

            draw.ellipse([330, 780, 370, 820], fill=cyan)
            draw.ellipse([430, 730, 470, 770], fill=purple)
            draw.ellipse([490, 738, 534, 782], fill=cyan)
            draw.ellipse([554, 730, 594, 770], fill=purple)
            draw.ellipse([654, 780, 694, 820], fill=cyan)

            # Save PNG
            output_path = Path(__file__).parent / "logo.png"
            img.save(output_path, 'PNG')
            print(f"✓ Created PNG using Pillow: {output_path}")

            with open(output_path, 'rb') as f:
                png_data = f.read()
                png_b64 = base64.b64encode(png_data).decode()
                print(f"✓ PNG Base64 (first 100 chars): {png_b64[:100]}...")
                return
        except ImportError:
            pass

    except Exception as e:
        print(f"⚠ Warning: {e}")

    print("\n" + "="*60)
    print("LOGO GENERATION COMPLETE")
    print("="*60)
    print("\nGenerated Files:")
    print("  • logo.svg - SVG vector logo (recommended for web)")
    print("  • logo-converter.html - Browser-based PNG converter")
    print("\nUsage:")
    print("  1. Add logo.svg to your Docusaurus project:")
    print("     static/img/logo.svg")
    print("  2. Or use in navbar configuration:")
    print("     logo: { src: '/img/logo.svg' }")
    print("\nFor PNG:")
    print("  • Open logo-converter.html in a browser")
    print("  • Click 'Download PNG' button")
    print("  • Place as: static/img/logo.png")
    print("\n" + "="*60)

if __name__ == "__main__":
    create_png_from_svg_data()
