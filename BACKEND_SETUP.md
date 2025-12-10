# Backend Setup & Running Guide

## Quick Start to Run RAG Chatbot Backend

### Prerequisites

1. **Python 3.11+** (installed)
2. **External Services** (must be configured):
   - OpenAI API account with GPT-4 access
   - Neon PostgreSQL database
   - Qdrant Cloud cluster

3. **Environment Variables** (must be set before running)

---

## Step 1: Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

**Expected output:**
```
Successfully installed fastapi==0.104.1 uvicorn[standard]==0.24.0 ...
```

---

## Step 2: Configure Environment Variables

### Option A: Using .env file (Recommended for Development)

Create `.env` file in the project root with:

```bash
# Copy template
cp .env.example .env

# Edit with your actual credentials
nano .env  # or use your editor
```

**Required variables to fill in:**

```env
# OpenAI API (Get from https://platform.openai.com/api-keys)
OPENAI_API_KEY=sk-your-actual-key-here
OPENAI_MODEL=gpt-4-turbo
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Qdrant Cloud (Get from https://cloud.qdrant.io)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key-here
QDRANT_COLLECTION_NAME=documentation_vectors

# Neon PostgreSQL (Get from https://neon.tech)
NEON_CONNECTION_STRING=postgresql://user:password@host/dbname?sslmode=require

# Frontend URL
FRONTEND_URL=http://localhost:3000
```

### Option B: Export Environment Variables

```bash
export OPENAI_API_KEY=sk-...
export QDRANT_URL=https://...
export NEON_CONNECTION_STRING=postgresql://...
# ... etc
```

---

## Step 3: Run the Backend Server

### Development Mode (with auto-reload)

```bash
cd backend
python -m uvicorn app.main:app --reload
```

**Expected output:**
```
INFO:     Uvicorn running on http://127.0.0.1:8000
INFO:     Application startup complete
```

### Production Mode (single process)

```bash
cd backend
python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
```

---

## Step 4: Verify Backend is Running

### Check Health Endpoint

```bash
curl http://localhost:8000/health
```

**Expected response (if services are configured correctly):**
```json
{
  "status": "ok",
  "timestamp": 1705315200.123,
  "checks": {
    "neon": true,
    "qdrant": true,
    "openai": true
  }
}
```

**If you see errors:**
- Check that OPENAI_API_KEY is correct
- Verify QDRANT_URL is accessible
- Confirm NEON_CONNECTION_STRING is valid

### Test Root Endpoint

```bash
curl http://localhost:8000/
```

**Expected response:**
```json
{
  "name": "RAG Chatbot Backend",
  "version": "1.0.0",
  "endpoints": {
    "health": "/health",
    "ask": "/ask",
    "chat_history": "/chat-history",
    "ingest": "/ingest"
  }
}
```

---

## Step 5: Test Core Functionality

### Test RAG Endpoint

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "selected_text": null
  }'
```

**Expected response:**
```json
{
  "answer": "ROS 2 is a middleware...",
  "sources": [
    {
      "title": "ROS 2 Basics",
      "section": "Introduction",
      "url": "/docs/ros2-basics"
    }
  ],
  "status": "success",
  "latency_ms": 1234
}
```

### Access API Documentation

Open browser and visit:
```
http://localhost:8000/docs
```

This opens interactive Swagger UI where you can test all endpoints.

---

## Troubleshooting

### Error: "Field required: openai_api_key"

**Solution:**
```bash
# Check if .env file exists
ls .env

# Make sure OPENAI_API_KEY is set
echo $OPENAI_API_KEY

# If empty, set it
export OPENAI_API_KEY=sk-your-key-here
python -m uvicorn app.main:app --reload
```

### Error: "Field required: neon_connection_string"

**Solution:**
```bash
# Verify PostgreSQL connection string is set
echo $NEON_CONNECTION_STRING

# Should look like:
# postgresql://user:password@host/dbname?sslmode=require
```

### Error: "Connection refused" for Qdrant

**Solution:**
1. Check that Qdrant cluster URL is correct
2. Verify Qdrant API key is valid
3. Ensure cluster is running

### Error: "Invalid API key provided" from OpenAI

**Solution:**
1. Get new API key from https://platform.openai.com/api-keys
2. Verify GPT-4 access is enabled on your OpenAI account
3. Make sure key starts with `sk-`

---

## Project Structure

```
backend/
├── app/
│   ├── __init__.py          # Package initialization
│   ├── main.py              # FastAPI application
│   ├── config.py            # Settings & configuration
│   ├── models.py            # Pydantic request/response models
│   ├── rag_pipeline.py      # RAG question-answering logic
│   ├── neon_client.py       # PostgreSQL client
│   ├── qdrant_client.py     # Vector database client
│   ├── cache.py             # In-memory caching
│   └── monitoring.py        # Logging & metrics
├── tests/                   # Unit and integration tests
├── migrations/              # Database migration scripts
├── main.py                  # (in root) Entry point if running directly
├── ingest.py                # Document ingestion script
├── migrate.py               # Database migration runner
├── requirements.txt         # Python dependencies
└── Dockerfile              # Docker containerization
```

---

## API Endpoints

| Method | Endpoint | Purpose |
|--------|----------|---------|
| GET | `/` | Root endpoint with service info |
| GET | `/health` | Health check for all services |
| POST | `/ask` | Ask a question about documentation |
| GET | `/chat-history?limit=50&offset=0` | Get chat history |
| GET | `/chat-history/{id}` | Get specific chat entry |
| POST | `/ingest` | Upload/process documents |

---

## Environment Variables Reference

| Variable | Required | Example | Description |
|----------|----------|---------|-------------|
| `OPENAI_API_KEY` | ✅ Yes | `sk-...` | OpenAI API authentication |
| `OPENAI_MODEL` | ✅ Yes | `gpt-4-turbo` | LLM model name |
| `OPENAI_EMBEDDING_MODEL` | ✅ Yes | `text-embedding-3-small` | Embedding model |
| `QDRANT_URL` | ✅ Yes | `https://...qdrant.io` | Vector database URL |
| `QDRANT_API_KEY` | ✅ Yes | `[api-key]` | Qdrant authentication |
| `NEON_CONNECTION_STRING` | ✅ Yes | `postgresql://...` | PostgreSQL connection |
| `FRONTEND_URL` | ⚠️ Development | `http://localhost:3000` | CORS allowed origin |
| `BACKEND_HOST` | ⚠️ Development | `127.0.0.1` | Server binding address |
| `BACKEND_PORT` | ⚠️ Development | `8000` | Server port |
| `LOG_LEVEL` | Optional | `INFO` | Logging level |
| `RAG_TOP_K` | Optional | `3` | Search result count |
| `RAG_MIN_SIMILARITY` | Optional | `0.5` | Min relevance threshold |

---

## Common Workflows

### 1. Local Development

```bash
# Terminal 1: Start backend
cd backend
python -m uvicorn app.main:app --reload

# Terminal 2: Test endpoints
curl http://localhost:8000/health
curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question":"What is ROS?"}'
```

### 2. Run Tests

```bash
cd backend
pytest tests/ -v                    # Run all tests
pytest tests/test_models.py -v      # Run specific test file
pytest --cov=app tests/             # Run with coverage
```

### 3. Database Initialization

```bash
cd backend
python migrate.py --up              # Apply migrations
python migrate.py --status          # Check migration status
python ingest.py --init-db --docs-path ../docs  # Ingest documents
```

### 4. Production Deployment

See `DEPLOYMENT.md` for complete deployment guide:
```bash
cat ../DEPLOYMENT.md
```

---

## Getting Help

**Check Logs:**
```bash
# View live logs from running backend
tail -f backend.log

# Or from terminal (if running in foreground)
# Logs appear in console
```

**Run Diagnostics:**
```bash
cd backend
python -c "from app.config import settings; print(settings)"
python -c "import app; print(app.__dict__)"
```

**Test Individual Components:**
```bash
python -c "
from app.config import settings
from app.neon_client import neon_client
import asyncio

async def test():
    await neon_client.connect()
    result = await neon_client.health_check()
    print(f'Neon health: {result}')
    await neon_client.disconnect()

asyncio.run(test())
"
```

---

## Next Steps

1. ✅ Install dependencies
2. ✅ Configure environment variables
3. ✅ Run backend server
4. ✅ Verify health endpoint
5. ✅ Test RAG endpoint
6. 👉 Check `DEPLOYMENT.md` for production setup
7. 👉 Check `OPERATIONS.md` for monitoring guide

---

**Status**: Backend system complete and ready for local development or production deployment!

For questions, check the documentation files or review the API at http://localhost:8000/docs

Last Updated: 2024-01-15
