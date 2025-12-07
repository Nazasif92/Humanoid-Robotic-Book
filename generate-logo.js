#!/usr/bin/env node

const fs = require('fs');
const path = require('path');

// Create SVG logo as base64 PNG equivalent data URL
const svgLogoXml = `<?xml version="1.0" encoding="UTF-8"?>
<svg width="1024" height="1024" viewBox="0 0 1024 1024" xmlns="http://www.w3.org/2000/svg">
  <!-- Background - transparent -->
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

  <!-- Outer tech circle border -->
  <circle cx="512" cy="512" r="490" class="tech-circle"/>

  <!-- TOP SECTION: AI Knowledge Nodes -->
  <!-- Knowledge connection lines -->
  <line x1="300" y1="200" x2="450" y2="280" class="knowledge-line"/>
  <line x1="450" y1="280" x2="600" y2="200" class="knowledge-line"/>
  <line x1="450" y1="280" x2="512" y2="450" class="knowledge-line"/>

  <!-- Knowledge nodes -->
  <circle cx="300" cy="200" r="24" class="knowledge-node"/>
  <circle cx="600" cy="200" r="24" class="knowledge-node"/>
  <circle cx="512" cy="320" r="32" class="knowledge-node-large"/>

  <!-- MIDDLE SECTION: Robot Arm -->
  <!-- Vertical arm -->
  <line x1="512" y1="400" x2="512" y2="620" class="robot-arm-v"/>
  <!-- Horizontal arm -->
  <line x1="512" y1="620" x2="720" y2="620" class="robot-arm-h"/>
  <!-- Central joint -->
  <circle cx="512" cy="620" r="18" class="joint"/>
  <!-- End effector -->
  <circle cx="720" cy="620" r="14" class="joint" fill="#00FFC8" opacity="0.8"/>

  <!-- BOTTOM SECTION: Agent Network (5 nodes) -->
  <!-- Agent connection lines -->
  <line x1="350" y1="800" x2="450" y2="750" class="agent-line"/>
  <line x1="450" y1="750" x2="512" y2="760" class="agent-line"/>
  <line x1="512" y1="760" x2="574" y2="750" class="agent-line"/>
  <line x1="574" y1="750" x2="674" y2="800" class="agent-line"/>

  <!-- Central bottom line -->
  <line x1="350" y1="800" x2="674" y2="800" class="agent-line"/>

  <!-- Agent nodes -->
  <circle cx="350" cy="800" r="20" class="agent-node"/>
  <circle cx="450" cy="750" r="20" class="agent-node-alt"/>
  <circle cx="512" cy="760" r="22" class="agent-node"/>
  <circle cx="574" cy="750" r="20" class="agent-node-alt"/>
  <circle cx="674" cy="800" r="20" class="agent-node"/>
</svg>`;

// Convert SVG to base64
const svgBase64 = Buffer.from(svgLogoXml).toString('base64');

console.log('SVG Logo Generated Successfully');
console.log('================================');
console.log('');
console.log('SVG Content (Base64):');
console.log(svgBase64);
console.log('');

// Create data URL
const dataUrl = `data:image/svg+xml;base64,${svgBase64}`;

// Save SVG file
const svgPath = path.join(__dirname, 'logo.svg');
fs.writeFileSync(svgPath, svgLogoXml);
console.log(`✓ Saved: ${svgPath}`);

// Create HTML file that can be used to convert SVG to PNG client-side
const htmlContent = `<!DOCTYPE html>
<html>
<head>
    <title>Logo PNG Converter</title>
</head>
<body>
    <h1>Humanoid-Robotic-Book Logo</h1>
    <p>Open this file in a browser to generate the PNG file.</p>

    <canvas id="canvas" width="1024" height="1024" style="border: 1px solid #ccc; display: block; margin: 20px 0;"></canvas>

    <button onclick="downloadPNG()">Download PNG</button>

    <script>
        const svgXml = \`${svgLogoXml.replace(/`/g, '\\`')}\`;

        const img = new Image();
        img.onload = function() {
            const canvas = document.getElementById('canvas');
            const ctx = canvas.getContext('2d');
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.drawImage(img, 0, 0);
        };

        const blob = new Blob([svgXml], { type: 'image/svg+xml' });
        const url = URL.createObjectURL(blob);
        img.src = url;

        function downloadPNG() {
            const canvas = document.getElementById('canvas');
            const link = document.createElement('a');
            link.href = canvas.toDataURL('image/png');
            link.download = 'logo.png';
            link.click();
        }
    </script>
</body>
</html>`;

const htmlPath = path.join(__dirname, 'logo-converter.html');
fs.writeFileSync(htmlPath, htmlContent);
console.log(`✓ Created: ${htmlPath} (Browser-based PNG converter)`);

console.log('');
console.log('To create the PNG logo:');
console.log('1. Option A (Recommended): Use Vercel Dashboard or static site - SVG displays natively');
console.log('2. Option B: Open logo-converter.html in a browser and click "Download PNG"');
console.log('3. Option C: Use the SVG directly in Docusaurus (logo.svg)');
console.log('');
