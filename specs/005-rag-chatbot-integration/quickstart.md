# Quickstart: RAG Chatbot Development

**Date**: 2025-12-07 | **Feature**: 005-rag-chatbot-integration

This guide helps you set up and test the RAG chatbot backend locally before deployment.

## Prerequisites

- Python 3.11+
- Neon PostgreSQL account (free tier: https://neon.tech)
- Qdrant account (free tier: https://qdrant.tech) OR local Qdrant Docker
- OpenAI API key (https://platform.openai.com/account/api-keys)
- Git, Node.js 18+ (for Docusaurus frontend)

## Step 1: Set Up Backend Environment

### 1.1 Clone & Navigate

```bash
cd /path/to/Humanoid-Robotic-Book
git checkout 005-rag-chatbot-integration
cd backend
```

### 1.2 Create Virtual Environment

```bash
python3.11 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 1.3 Install Dependencies

```bash
pip install -r requirements.txt
```

## Step 2: Configure Environment Variables

### 2.1 Copy Example File

```bash
cp ../.env.example .env
```

### 2.2 Fill in Values

Edit `.env`:

```dotenv
# OpenAI
OPENAI_API_KEY=sk-xxx...
OPENAI_MODEL=gpt-4-turbo-preview
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Qdrant (local Docker OR cloud)
QDRANT_URL=http://localhost:6333  # Local
# QDRANT_URL=https://xxx-xxx-xxx.qdrant.io  # Cloud
QDRANT_API_KEY=  # Empty for local; set for cloud
QDRANT_COLLECTION_NAME=humanoid_docs

# Neon PostgreSQL
NEON_CONNECTION_STRING=postgresql://user:password@ep-xxx.neon.tech/dbname?sslmode=require

# Backend
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
FRONTEND_URL=http://localhost:3000  # Local Docusaurus dev
# FRONTEND_URL=https://humanoid-robotic-book.vercel.app  # Production

# Ingestion
DOCS_PATH=../docs  # Relative to backend/
```

## Step 3: Set Up Databases

### 3.1 Local Qdrant (Optional)

```bash
docker run -p 6333:6333 \
  -e QDRANT__HTTP__API_KEY=qdrant_key \
  qdrant/qdrant:latest
```

Visit `http://localhost:6333/dashboard` to verify.

### 3.2 Neon PostgreSQL

1. Create free account at https://neon.tech
2. Create a new project (e.g., "humanoid-chatbot")
3. Copy connection string: `postgresql://user:password@ep-xxx.neon.tech/neondb`
4. Paste into `.env` as `NEON_CONNECTION_STRING`

## Step 4: Initialize Database & Ingest Documents

### 4.1 Create Tables

```bash
python ingest.py --init-db
```

Expected output:
```
✅ Database tables created successfully
✅ Qdrant collection 'humanoid_docs' initialized
```

### 4.2 Ingest Documents

```bash
python ingest.py
```

Expected output:
```
📚 Ingesting documents from ../docs/
  📄 Processing docs/intro.md
  📄 Processing docs/chapters/ros2-nervous-system.md
  ...
✅ Ingestion complete!
   - 8 documents processed
   - 127 chunks created
   - Duration: 45.3 seconds
```

## Step 5: Run Backend Server

```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Expected output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete
```

## Step 6: Test API Endpoints

### 6.1 Health Check

```bash
curl -X GET http://localhost:8000/health
```

Expected response:
```json
{
  "status": "ok",
  "timestamp": "2025-12-07T12:00:00Z",
  "checks": {
    "qdrant": true,
    "neon": true,
    "openai": true
  }
}
```

### 6.2 Ask a Question

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2 and why is it important for robotics?"
  }'
```

Expected response:
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is a middleware framework...",
  "sources": [
    {
      "title": "Module 1: ROS 2 — Robotic Nervous System",
      "section": "chapters",
      "url": "/docs/chapters/ros2-nervous-system",
      "chunk_text": "ROS 2 is a flexible framework..."
    }
  ],
  "status": "success",
  "latency_ms": 2145
}
```

### 6.3 Ask with Selected Text Context

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Can you explain this further?",
    "selected_text": "ROS 2 uses a publish-subscribe architecture..."
  }'
```

## Step 7: Run Tests

```bash
pytest tests/ -v
```

Expected output:
```
tests/test_api_endpoints.py::test_health_check PASSED
tests/test_api_endpoints.py::test_ask_valid_question PASSED
tests/test_rag_pipeline.py::test_embedding_generation PASSED
tests/test_ingestion.py::test_markdown_parsing PASSED
===== 12 passed in 2.34s =====
```

## Step 8: Set Up Frontend (Local Development)

### 8.1 Install Docusaurus Dependencies

```bash
cd ../
npm install
```

### 8.2 Create Chatbot Page

Create `src/pages/chatbot.jsx` (see implementation tasks for full code).

### 8.3 Update Navbar

Edit `docusaurus.config.js`:

```js
{
  label: 'Chatbot',
  to: '/chatbot',
  position: 'right',
}
```

### 8.4 Start Docusaurus Dev Server

```bash
npm run start
```

Visit `http://localhost:3000/chatbot` and test the chatbot UI.

## Step 9: Deploy Backend to Railway

### 9.1 Create Railway Account

Visit https://railway.app and sign up with GitHub.

### 9.2 Connect Repository

1. Create new project → "Deploy from GitHub"
2. Select `Humanoid-Robotic-Book` repository
3. Railway auto-detects `railway.json` in root

### 9.3 Set Environment Variables

In Railway dashboard:

```
OPENAI_API_KEY=sk-xxx...
NEON_CONNECTION_STRING=postgresql://...
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=xxx...
FRONTEND_URL=https://humanoid-robotic-book.vercel.app
```

### 9.4 Deploy

```bash
git push origin 005-rag-chatbot-integration
```

Railway auto-deploys. Get backend URL from Railway dashboard (e.g., `https://rag-chatbot-backend.railway.app`).

## Step 10: Update Frontend to Point to Production Backend

Edit `src/pages/chatbot.jsx`:

```jsx
const BACKEND_URL = process.env.NODE_ENV === 'production'
  ? 'https://rag-chatbot-backend.railway.app'
  : 'http://localhost:8000';
```

## Troubleshooting

### "Connection refused" on Qdrant

- Ensure Docker container is running: `docker ps`
- Or set `QDRANT_URL` to your cloud instance

### "OpenAI API rate limited"

- Check API key: `echo $OPENAI_API_KEY`
- Reduce concurrent ingestion: `python ingest.py --batch-size 10`

### "Neon connection timeout"

- Verify connection string has `?sslmode=require`
- Check IP allowlist in Neon dashboard

### Frontend not calling backend

- Check CORS settings in `app/main.py` match `FRONTEND_URL`
- Check browser console for CORS errors

## Next Steps

- Run `/sp.tasks` to generate detailed implementation tasks
- Create PR from `005-rag-chatbot-integration` to `main`
- Set up CI/CD for backend tests in GitHub Actions
- Configure scheduled ingestion job in Railway

---

**Need help?** Check the full spec: [spec.md](spec.md) | Full plan: [plan.md](plan.md)
