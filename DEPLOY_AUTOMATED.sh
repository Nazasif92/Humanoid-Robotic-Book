#!/bin/bash

# Humanoid-Robotic-Book - Automated Deployment Script
# This script handles end-to-end deployment to Vercel
# Usage: VERCEL_TOKEN=your_token ./DEPLOY_AUTOMATED.sh

set -e

PROJECT_DIR="D:\GIAIC\AI agent\Hakathon\Hakathon-01\Humanoid-Robotic-Book"
cd "$PROJECT_DIR"

echo "=========================================="
echo "Humanoid-Robotic-Book - Automated Deploy"
echo "=========================================="
echo ""

# Step 1: Verify Vercel CLI
echo "Step 1: Verifying Vercel CLI..."
if ! command -v vercel &> /dev/null; then
  echo "Installing Vercel CLI..."
  npm install -g vercel
fi
vercel --version
echo "✓ Vercel CLI ready"
echo ""

# Step 2: Verify build directory
echo "Step 2: Verifying build directory..."
if [ ! -d build ] || [ $(find build -type f | wc -l) -lt 10 ]; then
  echo "Building project..."
  npm run build
fi
echo "✓ Build directory ready ($(find build -type f | wc -l) files)"
echo ""

# Step 3: Check for token
echo "Step 3: Checking for Vercel token..."
if [ -z "$VERCEL_TOKEN" ]; then
  echo "ERROR: VERCEL_TOKEN environment variable not set"
  echo "Usage: VERCEL_TOKEN=your_token ./DEPLOY_AUTOMATED.sh"
  exit 1
fi
echo "✓ Token configured"
echo ""

# Step 4: Deploy
echo "Step 4: Deploying to production..."
export VERCEL_TOKEN
vercel --prod --yes 2>&1 | tee deployment.log

# Step 5: Extract and display URL
echo ""
echo "=========================================="
echo "Deployment Complete!"
echo "=========================================="
echo ""

# Get deployment URL from logs
PROD_URL=$(grep -oP 'https://[a-zA-Z0-9.-]+\.vercel\.app' deployment.log | head -1)

if [ -n "$PROD_URL" ]; then
  echo "✓ Production URL: $PROD_URL"
else
  echo "✓ Deployment submitted"
  echo "Check Vercel Dashboard: https://vercel.com/dashboard"
fi

echo ""
echo "Project is now live!"
