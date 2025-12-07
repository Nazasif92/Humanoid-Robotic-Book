# RAG Chatbot Deployment Guide

Complete guide for deploying the Humanoid Robotics Book RAG chatbot system to production on Railway and Vercel.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Prerequisites](#prerequisites)
3. [Environment Setup](#environment-setup)
4. [Backend Deployment (Railway)](#backend-deployment-railway)
5. [Frontend Deployment (Vercel)](#frontend-deployment-vercel)
6. [Database Setup (Neon PostgreSQL + Qdrant)](#database-setup)
7. [Monitoring & Operations](#monitoring--operations)
8. [Troubleshooting](#troubleshooting)

---

## Architecture Overview

```
User Request
    ↓
Vercel (Frontend)
    ↓ HTTPS
Railway (FastAPI Backend)
    ├─ OpenAI API (GPT-4)
    ├─ Qdrant (Vector Search)
    └─ Neon PostgreSQL (Metadata)
```

**Key Components:**
- **Frontend**: Docusaurus 3.x with React chatbot page on Vercel
- **Backend**: FastAPI with RAG pipeline on Railway
- **Vector DB**: Qdrant Cloud for semantic search
- **Metadata DB**: Neon Serverless PostgreSQL
- **LLM**: OpenAI GPT-4 Turbo

---

## Prerequisites

### Required Accounts
- [ ] [Railway.app](https://railway.app) account
- [ ] [Vercel](https://vercel.com) account
- [ ] [Neon](https://neon.tech) account
- [ ] [Qdrant Cloud](https://cloud.qdrant.io) account
- [ ] [OpenAI API](https://platform.openai.com/api-keys) account with GPT-4 access

### Required Tools
```bash
npm install -g vercel
pip install -r backend/requirements.txt
```

### Repository Setup
```bash
git clone https://github.com/Nazasif92/Humanoid-Robotic-Book.git
cd Humanoid-Robotic-Book
git checkout 005-rag-chatbot-integration
```

---

## Environment Setup

### 1. Create Production Environment Files

**Copy template and configure:**
```bash
cp .env.production .env.production.local
# Edit with your actual credentials
```

**Essential variables to configure:**

```env
# OpenAI API
OPENAI_API_KEY=sk-... (Get from https://platform.openai.com/api-keys)

# Neon PostgreSQL
NEON_CONNECTION_STRING=postgresql://user:pass@host/dbname?sslmode=require

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=...

# Frontend CORS
FRONTEND_URL=https://your-domain.vercel.app
```

### 2. Database Initialization

#### Create Neon PostgreSQL Database

1. Go to [Neon Console](https://console.neon.tech)
2. Create a new project
3. Create a new database for the application
4. Copy connection string: `postgresql://...`
5. Run migrations:

```bash
python backend/migrate.py --up
```

#### Setup Qdrant Cloud

1. Go to [Qdrant Cloud Console](https://cloud.qdrant.io)
2. Create a new cluster
3. Create a new collection named `documentation_vectors`:
   - Vector size: 1536 (for OpenAI text-embedding-3-small)
   - Distance metric: Cosine
4. Copy API key and cluster URL

---

## Backend Deployment (Railway)

### Step 1: Connect GitHub Repository

1. Go to [Railway.app](https://railway.app)
2. Click "New Project" → "Deploy from GitHub repo"
3. Connect to your GitHub account
4. Select the `Humanoid-Robotic-Book` repository
5. Select branch: `005-rag-chatbot-integration`

### Step 2: Configure Environment Variables

In Railway project settings, set all variables from `.env.production`:

**Critical variables:**
```
OPENAI_API_KEY=sk-...
NEON_CONNECTION_STRING=postgresql://...
QDRANT_URL=https://...
QDRANT_API_KEY=...
FRONTEND_URL=https://your-domain.vercel.app
```

**Optional (defaults work for MVP):**
```
LOG_LEVEL=INFO
RAG_TOP_K=3
RAG_MIN_SIMILARITY=0.5
RATE_LIMIT_REQUESTS=20
RATE_LIMIT_PERIOD=60
```

### Step 3: Deploy

Railway automatically detects the `Dockerfile` and `railway.json` and deploys:

```bash
# Manual deployment (if needed)
railway deploy
```

**Expected output:**
```
✓ Building Docker image
✓ Pushing to Railway registry
✓ Service deployed to https://your-service.up.railway.app
```

### Step 4: Verify Backend Health

```bash
curl https://your-railway-url/health
# Expected response:
# {"status":"ok","checks":{"neon":true,"qdrant":true,"openai":true},...}
```

---

## Frontend Deployment (Vercel)

### Step 1: Connect GitHub Repository

1. Go to [Vercel.com](https://vercel.com)
2. Click "New Project"
3. Import `Humanoid-Robotic-Book` repository
4. Select branch: `005-rag-chatbot-integration`

### Step 2: Configure Environment Variables

In Vercel project settings, set:

```
NEXT_PUBLIC_BACKEND_URL=https://your-railway-url
NEXT_PUBLIC_FRONTEND_URL=https://your-domain.vercel.app
```

### Step 3: Deploy

```bash
vercel --prod
```

**Expected output:**
```
✓ Vercel CLI 32.0.0
✓ Production deployment
✓ Live at https://humanoid-robotic-book.vercel.app
```

### Step 4: Verify Frontend

1. Visit https://humanoid-robotic-book.vercel.app
2. Click "Chatbot" in navbar
3. Ask a test question: "What is ROS 2?"
4. Verify answer and sources appear

---

## Database Setup

### Initialize Databases

#### 1. Create Neon Tables

```bash
# Run migrations (creates all tables)
python backend/migrate.py --up

# Verify
python backend/migrate.py --status
```

Expected tables:
- `documents` - Document metadata
- `chunks` - Document chunks with embeddings
- `chat_logs` - Chat interaction history
- `schema_migrations` - Migration tracking

#### 2. Ingest Documentation

```bash
# Run ingestion pipeline
python backend/ingest.py --init-db --docs-path ../docs

# Expected output:
# ============================================================
# INGESTION SUMMARY
# ============================================================
# Status: SUCCESS
# Documents processed: 42
# Chunks created: 1247
# ============================================================
```

### Database Backups

**Automatic backups** (handled by Neon):
- Hourly backups retained for 7 days
- Daily backups retained for 30 days

**Manual backup:**
```bash
pg_dump $NEON_CONNECTION_STRING > backup_$(date +%Y%m%d_%H%M%S).sql
```

**Restore from backup:**
```bash
psql $NEON_CONNECTION_STRING < backup_20240115_120000.sql
```

---

## Monitoring & Operations

### Health Checks

**Backend health endpoint:**
```bash
curl https://your-railway-url/health

# Response:
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

**Check logs:**
```bash
# Railway logs
railway logs

# View specific service
railway logs --service backend
```

### Performance Monitoring

**Key metrics to monitor:**
- API response latency (p95: < 3s)
- Vector search latency (p95: < 500ms)
- Database connection pool utilization
- Token usage vs quota
- Error rate (target: < 1%)

**Logging:**
- All requests logged with latency
- Errors logged with full stack trace
- Background tasks logged asynchronously

### Common Operations

#### View Recent Chats
```bash
# Via psql
psql $NEON_CONNECTION_STRING -c "SELECT * FROM chat_logs LIMIT 10;"

# Or via Python
python -c "
import asyncio
from app.neon_client import neon_client
neon_client.connect()
total, chats = asyncio.run(neon_client.get_chat_history(limit=10))
print(f'Total: {total}')
for chat in chats: print(f'  {chat}')
"
```

#### Re-index Documents
```bash
# Clear existing vectors
python -c "
from app.qdrant_client import qdrant_client
qdrant_client.connect()
qdrant_client.delete_by_payload({'doc_id': {'$exists': True}})
"

# Re-ingest
python backend/ingest.py --docs-path ../docs
```

#### Rotate API Keys
1. Update `OPENAI_API_KEY` in Railway environment
2. Update `QDRANT_API_KEY` in Railway environment
3. Redeploy application
4. Verify health check passes

---

## Troubleshooting

### Backend Issues

#### Issue: "502 Bad Gateway"
**Symptoms:** Frontend receives 502 error

**Diagnosis:**
```bash
# Check railway logs
railway logs

# Verify database connection
curl https://your-railway-url/health
```

**Resolution:**
1. Check `NEON_CONNECTION_STRING` is valid
2. Verify Neon database is online
3. Check Qdrant cluster is accessible
4. Review recent code changes

#### Issue: "Unauthorized" from OpenAI
**Symptoms:** "Invalid API key provided" errors

**Resolution:**
1. Verify `OPENAI_API_KEY` in Railway settings
2. Check API key hasn't exceeded usage quota
3. Generate new key at https://platform.openai.com/api-keys
4. Update Railway environment and redeploy

#### Issue: Slow Response Times (> 5s)
**Symptoms:** Chatbot responses take very long

**Diagnosis:**
```bash
# Check OpenAI status
curl https://status.openai.com/api/v2/status.json

# Verify Qdrant performance
# Check Railway metrics dashboard
```

**Resolution:**
1. Check OpenAI API status
2. Verify Qdrant cluster is not overloaded
3. Review `RAG_TOP_K` setting (reduce from 3 to 1-2)
4. Check database query performance

### Frontend Issues

#### Issue: "Chatbot not loading"
**Symptoms:** Blank white screen on /chatbot page

**Diagnosis:**
```bash
# Check browser console for errors
# (F12 → Console tab)
```

**Resolution:**
1. Verify `NEXT_PUBLIC_BACKEND_URL` is correct
2. Check CORS is enabled in backend
3. Verify Vercel deployment is complete
4. Clear browser cache (Ctrl+Shift+Delete)

#### Issue: "Cannot reach backend"
**Symptoms:** "Error: Failed to connect to backend"

**Resolution:**
1. Verify Railway URL is correct in Vercel env
2. Check Railway service is running
3. Verify CORS `FRONTEND_URL` matches Vercel domain
4. Test backend directly: `curl https://your-railway-url/health`

### Database Issues

#### Issue: "Connection pool exhausted"
**Symptoms:** "FATAL: remaining connection slots are reserved"

**Resolution:**
```bash
# Increase pool size
# In Railway: set NEON_POOL_MAX_SIZE=30

# Or reduce connections from application
# Check for connection leaks in code
```

#### Issue: "Vector search returns no results"
**Symptoms:** All questions return "I don't have information..."

**Resolution:**
1. Verify documents are ingested:
   ```bash
   psql $NEON_CONNECTION_STRING -c "SELECT COUNT(*) FROM chunks;"
   ```
2. Check Qdrant collection has vectors:
   ```bash
   # Via Qdrant console or API
   ```
3. Lower `RAG_MIN_SIMILARITY` threshold temporarily
4. Re-run ingestion if needed

---

## Scaling & Optimization

### High Traffic Optimization

For > 100 requests/minute:

1. **Increase Neon pool:**
   ```
   NEON_POOL_MAX_SIZE=30
   NEON_POOL_MIN_SIZE=10
   ```

2. **Scale Qdrant cluster** to Professional plan

3. **Enable caching:**
   ```
   ENABLE_REDIS_CACHING=true
   REDIS_URL=redis://...
   ```

4. **Rate limit adjustments:**
   ```
   RATE_LIMIT_REQUESTS=100
   RATE_LIMIT_PERIOD=60
   ```

### Cost Optimization

- **Neon**: Use Autoscaling (CPU: 0.5-2, RAM: 1-4GB)
- **Qdrant**: Start with Standard plan, upgrade as needed
- **OpenAI**: Use gpt-3.5-turbo if acceptable for use case
- **Railway**: Use GitHub Actions for CI/CD instead of per-push builds

---

## Security Checklist

- [ ] All API keys stored in Railway secrets (not in code)
- [ ] CORS restricted to Vercel domain only
- [ ] SSL/TLS enabled on all connections
- [ ] Database passwords never logged
- [ ] Rate limiting enabled (20 req/min)
- [ ] Input validation on all requests
- [ ] Error messages don't expose sensitive info
- [ ] Database backups retained for 30 days
- [ ] Access logs monitored for suspicious activity

---

## Support & Contact

For issues or questions:
1. Check [GitHub Issues](https://github.com/Nazasif92/Humanoid-Robotic-Book/issues)
2. Review logs in Railway dashboard
3. Test backend health: `GET /health`
4. Check OpenAI API status
5. Verify database connectivity

---

**Last Updated:** 2024-01-15
**Deployment Version:** 1.0.0
**Maintainer:** AI Engineering Team
