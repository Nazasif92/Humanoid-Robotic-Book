#!/bin/bash

# Deployment script for Docusaurus Vercel deployment

echo "Starting deployment to Vercel..."

# 1. Ensure you have the latest code
git status
echo "Make sure your local branch is up to date with remote before deployment"

# 2. Verify build works correctly
echo "Building the Docusaurus site..."
npm run build

# 3. Deploy to Vercel
echo "Deploying to Vercel..."
npx vercel --prod

# 4. Set environment variables in Vercel dashboard after deployment
echo "After deployment, set the following environment variables in the Vercel dashboard:"
echo "1. Go to your Vercel project dashboard"
echo "2. Navigate to Settings > Environment Variables"
echo "3. Add the following variable:"
echo "   Key: API_BASE_URL"
echo "   Value: <your-backend-API-URL>"

# 5. Verify deployment
echo "Deployment completed. Verify the site works by accessing the deployed URL."