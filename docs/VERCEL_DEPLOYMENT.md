# Vercel Deployment Guide

This document explains how to deploy the Docusaurus site with integrated chatbot to Vercel.

## Prerequisites

- Vercel account
- Vercel CLI installed (`npm install -g vercel`)
- Git repository connected to your Vercel account

## Deployment Steps

1. **Prepare the Environment**
   - Ensure your `vercel.json` file is properly configured with the correct API rewrites
   - Verify that the chatbot component uses `process.env.API_BASE_URL`
   - Make sure all dependencies are installed (`npm install`)

2. **Build the Project**
   ```bash
   npm run build
   ```

3. **Deploy to Vercel**
   ```bash
   vercel --prod
   ```

4. **Configure Environment Variables**
   After deployment, go to your Vercel project dashboard and set the following environment variable:
   - Key: `API_BASE_URL`
   - Value: Your backend API URL (e.g., `https://your-backend-project-name-production.up.railway.app`)

5. **Verify the Deployment**
   - Access your deployed site URL
   - Test the chatbot functionality by submitting a query
   - Verify that responses are returned correctly

## Configuration Notes

- The `vercel.json` file is configured with proper API rewrites to handle backend communication
- The output directory is set to `build` which matches Docusaurus's default build output
- CORS headers are configured to allow communication between frontend and backend

## Troubleshooting

- If the chatbot doesn't work, verify that `API_BASE_URL` is correctly set in Vercel environment variables
- Check browser console for any CORS errors
- Ensure your backend API is accessible and responding to requests