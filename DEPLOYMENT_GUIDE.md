# Production Deployment Guide: Docusaurus + RAG Chatbot

## Overview
This guide provides step-by-step instructions to deploy your Docusaurus frontend with embedded RAG chatbot to Vercel, ensuring it connects successfully to the FastAPI RAG backend and Qdrant vector database.

## Prerequisites
- Vercel account (https://vercel.com)
- Railway account (https://railway.app) - for backend
- Qdrant Cloud account (https://qdrant.tech)
- OpenAI API key
- GitHub repository with your code

## Step 1: Deploy Backend to Railway

1. Navigate to your backend directory:
```bash
cd backend
```

2. Install Railway CLI:
```bash
npm install -g @railway/cli
```

3. Login to Railway:
```bash
railway login
```

4. Create a new project:
```bash
railway init
```

5. Set environment variables in Railway dashboard:
```
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_cloud_api_key_here
QDRANT_COLLECTION_NAME=your_collection_name
PORT=8000
ENVIRONMENT=production
ALLOWED_ORIGINS=https://your-frontend-project-name.vercel.app
```

6. Deploy to Railway:
```bash
railway up
```

7. Note your Railway backend URL (format: `https://your-project-name-production.up.railway.app`)

## Step 2: Configure Environment Variables

### For Local Development (.env file):
```env
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_cloud_api_key_here
QDRANT_COLLECTION_NAME=your_collection_name

# Backend Configuration
PORT=8000
ENVIRONMENT=development
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001,https://your-frontend-project-name.vercel.app

# Frontend Configuration
NEXT_PUBLIC_API_BASE_URL=http://localhost:8000
NEXT_PUBLIC_CHATBOT_ENABLED=true
```

### For Vercel Deployment:
Add these in your Vercel dashboard under Settings > Environment Variables:
```
NEXT_PUBLIC_API_BASE_URL=https://your-backend-project-name-production.up.railway.app
NEXT_PUBLIC_CHATBOT_ENABLED=true
```

### For Railway Backend:
Add these in your Railway dashboard under Settings > Variables:
```
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_cloud_api_key_here
QDRANT_COLLECTION_NAME=your_collection_name
PORT=8000
ENVIRONMENT=production
ALLOWED_ORIGINS=https://your-frontend-project-name.vercel.app
```

## Step 3: Deploy Frontend to Vercel

1. Install Vercel CLI:
```bash
npm install -g vercel
```

2. Login to Vercel:
```bash
vercel login
```

3. Navigate to project root:
```bash
cd /path/to/your/project
```

4. Deploy to Vercel:
```bash
vercel --prod
```

5. When prompted:
   - Set up and deploy? `Y`
   - Which scope? Select your account
   - Link to existing project? `N` (create new)
   - What's your project's name? Use default or enter custom
   - In which directory is your code located? `.`
   - Configuration: Accept defaults

6. Add environment variables in Vercel dashboard:
   - Go to your project dashboard
   - Navigate to Settings > Environment Variables
   - Add the Vercel environment variables listed above

## Step 4: Configure CORS in FastAPI Backend

Update your FastAPI backend (in `backend/app/main.py` or similar):

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os

app = FastAPI()

# CORS configuration for production
allowed_origins = os.getenv("ALLOWED_ORIGINS", "http://localhost:3000").split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Add this for production
    allow_origin_regex=r"https://.*\.vercel\.app"
)

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "rag-backend"}

@app.post("/ask")
async def ask_question(request: Request):
    # Your RAG logic here
    pass
```

## Step 5: Local Build Instructions

To build locally for testing:
```bash
# Install dependencies
npm install

# Build for production
npm run build

# Serve locally to test
npm run serve
```

## Step 6: Production Build Process

The production build is automatically handled by Vercel when you deploy, but the process is:

1. Vercel runs `npm run build` (as specified in vercel.json)
2. The build output is placed in the `build` directory
3. Static files are served from the `build` directory
4. API requests are proxied to the backend using the rewrites in vercel.json

## Step 7: Verification Steps

### API Test (Backend)
```bash
# Test health endpoint
curl https://your-backend-project-name-production.up.railway.app/health

# Expected response:
# {"status": "healthy", "service": "rag-backend"}
```

### Frontend Test
1. Visit your Vercel deployment URL: `https://your-project-name.vercel.app`
2. Navigate to the chatbot page: `/chatbot`
3. Test the chat functionality by asking a question
4. Check browser console for any errors

### Integration Test
```bash
# Test API call from frontend origin
curl -X POST \
  -H "Content-Type: application/json" \
  -H "Origin: https://your-project-name.vercel.app" \
  -d '{"question": "What is this project about?"}' \
  https://your-backend-project-name-production.up.railway.app/ask
```

### End-to-End Test
1. Go to your deployed site
2. Use the chatbot to ask a question related to your documentation
3. Verify that:
   - The question is sent to the backend successfully
   - The response includes relevant information
   - Sources are properly cited
   - No CORS errors occur

## Troubleshooting

### Common Issues:

1. **CORS Errors**: Ensure your Railway backend allows your Vercel domain
2. **API Connection Failures**: Verify environment variables are set correctly
3. **Build Failures**: Check that all dependencies are properly configured

### Deployment Verification Checklist:

- [ ] Frontend successfully deployed to Vercel
- [ ] Backend successfully deployed to Railway
- [ ] Environment variables properly configured on both platforms
- [ ] CORS configured to allow Vercel domain
- [ ] API health check returns healthy status
- [ ] Chat functionality works end-to-end
- [ ] All required environment variables are set
- [ ] Auto-deployment configured for both platforms
- [ ] SSL/HTTPS is active on both domains
- [ ] Error handling works properly in production

## Deploy Report Template

### Build Logs:
- Frontend build completed successfully
- Backend deployed to Railway
- No errors in build process

### Environment Configuration:
- Frontend: Vercel with NEXT_PUBLIC_API_BASE_URL pointing to Railway backend
- Backend: Railway with Qdrant and OpenAI credentials
- CORS: Configured for Vercel domain access

### Verification Results:
- Health check: ✅ Passed
- API connectivity: ✅ Confirmed
- Chat functionality: ✅ Working
- Source citations: ✅ Present
- Response quality: ✅ Satisfactory