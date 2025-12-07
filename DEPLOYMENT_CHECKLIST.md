# Production Deployment Checklist

**Start Date**: _____________
**Deployment Environment**: Production
**Deployed By**: _____________

---

## Pre-Deployment: Infrastructure Setup (30 minutes)

### Step 1: Create External Accounts & Services

#### OpenAI API
- [ ] Go to https://platform.openai.com/api-keys
- [ ] Create API key (should already exist)
- [ ] Verify GPT-4 access is enabled
- [ ] Copy API key: `sk-...`
- [ ] Save to secure location (password manager)

#### Neon PostgreSQL
- [ ] Go to https://console.neon.tech
- [ ] Create new project (if not exists)
- [ ] Create database for chatbot
- [ ] Go to Connection String → Copy connection string
- [ ] Format: `postgresql://user:password@host/dbname?sslmode=require`
- [ ] Save connection string securely

#### Qdrant Cloud
- [ ] Go to https://cloud.qdrant.io
- [ ] Create new cluster (Standard plan recommended)
- [ ] Wait for cluster to initialize (2-3 minutes)
- [ ] Go to API Keys → Create API key
- [ ] Copy API key
- [ ] Copy cluster URL (https://your-cluster-name.qdrant.io)
- [ ] Note collection name: `documentation_vectors`

#### Railway.app
- [ ] Go to https://railway.app
- [ ] Create new account or login
- [ ] Create new project
- [ ] Connect GitHub repository
- [ ] Select branch: `005-rag-chatbot-integration`

#### Vercel
- [ ] Go to https://vercel.com
- [ ] Create new account or login
- [ ] Create new project
- [ ] Import GitHub repository
- [ ] Select branch: `005-rag-chatbot-integration`

#### GitHub
- [ ] Create personal access token (Settings → Developer settings → Personal access tokens)
- [ ] Scopes: `repo`, `workflow`, `read:packages`
- [ ] Save token securely

---

## Step 2: Configure Environment Variables

### Create Railway Secrets

In Railway dashboard → Project → Settings → Environment:

```
# OpenAI Configuration
OPENAI_API_KEY=sk-[your-key-here]
OPENAI_MODEL=gpt-4-turbo
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
OPENAI_TIMEOUT=30

# Neon PostgreSQL
NEON_CONNECTION_STRING=postgresql://user:password@host/dbname?sslmode=require
NEON_POOL_MIN_SIZE=1
NEON_POOL_MAX_SIZE=20

# Qdrant Vector Database
QDRANT_URL=https://[cluster-name].qdrant.io
QDRANT_API_KEY=[your-api-key]
QDRANT_COLLECTION_NAME=documentation_vectors

# Frontend
FRONTEND_URL=https://[your-domain].vercel.app

# RAG Configuration
RAG_TOP_K=3
RAG_MIN_SIMILARITY=0.5

# Rate Limiting
RATE_LIMIT_REQUESTS=20
RATE_LIMIT_PERIOD=60

# Logging
LOG_LEVEL=INFO
```

**Checklist:**
- [ ] All variables filled in with actual values
- [ ] No placeholder values remaining
- [ ] API keys are correct format (not truncated)
- [ ] Verified database URL is accessible

### Create Vercel Secrets

In Vercel dashboard → Settings → Environment Variables:

```
NEXT_PUBLIC_BACKEND_URL=https://[railway-deployment-url]
NEXT_PUBLIC_FRONTEND_URL=https://[your-domain].vercel.app
```

**Checklist:**
- [ ] Backend URL matches deployed Railway service
- [ ] Frontend URL matches Vercel domain
- [ ] Both URLs are complete HTTPS URLs

---

## Step 3: Deploy Backend to Railway

### Option A: Using Railway Dashboard

1. Go to Railway project dashboard
2. Click "Deploy" button
3. Select branch: `005-rag-chatbot-integration`
4. Wait for build to complete (3-5 minutes)

**Status indicators:**
- `Building...` → Docker image being created
- `Deploying...` → Image being pushed to Railway
- `Running` → Service is live

### Option B: Using Railway CLI

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login to Railway
railway login

# Link to your project
railway link --project [project-id]

# Deploy
railway deploy
```

### Verify Deployment

```bash
# Get Railway URL
railway open  # Opens dashboard, look for URL

# Test health endpoint
curl https://[railway-url]/health
# Expected response:
# {"status":"ok","checks":{"neon":true,"qdrant":true,"openai":true},...}
```

**Checklist:**
- [ ] Build completed successfully (no errors)
- [ ] Health endpoint returns status: "ok"
- [ ] All service checks pass (neon, qdrant, openai)
- [ ] Copy deployed URL for next step

---

## Step 4: Initialize Database

### Run Migrations

```bash
# Get Railway service URL
BACKEND_URL="https://[your-railway-url]"

# Connect to database
export NEON_CONNECTION_STRING="postgresql://..."

# Run migrations
python backend/migrate.py --up

# Verify
python backend/migrate.py --status
```

**Expected output:**
```
Applied (1):
  ✓ 001_initial_schema

Pending (0):
```

### Ingest Documentation

```bash
# Run ingestion pipeline
python backend/ingest.py --init-db --docs-path ./docs

# Expected output:
# ============================================================
# INGESTION SUMMARY
# ============================================================
# Status: SUCCESS
# Documents processed: 42
# Chunks created: 1247
# ============================================================
```

**Checklist:**
- [ ] Migrations applied successfully
- [ ] No SQL errors
- [ ] Documents ingested without errors
- [ ] Total chunks > 100 (indicates successful ingestion)

---

## Step 5: Deploy Frontend to Vercel

### Option A: Using Vercel Dashboard

1. Go to Vercel project dashboard
2. Click "Deploy" button
3. Select branch: `005-rag-chatbot-integration`
4. Wait for build to complete (2-3 minutes)

### Option B: Using Vercel CLI

```bash
# Install Vercel CLI
npm install -g vercel

# Deploy to production
vercel --prod

# Follow prompts:
# - Use existing project? Yes
# - Project name: humanoid-robotic-book
```

### Verify Frontend Deployment

```bash
# Test frontend URL
curl https://[vercel-url]

# Check health endpoint through frontend
curl https://[vercel-url]/chatbot
```

**Checklist:**
- [ ] Build completed without errors
- [ ] Frontend URL is live
- [ ] Chatbot page loads correctly
- [ ] No CORS errors in console

---

## Step 6: End-to-End Testing

### Test Backend

```bash
# Test health check
curl https://[railway-url]/health

# Test RAG endpoint
curl -X POST https://[railway-url]/ask \
  -H "Content-Type: application/json" \
  -d '{"question":"What is ROS 2?"}'

# Expected response:
# {"answer":"...","sources":[...],"status":"success","latency_ms":1234}
```

### Test Frontend

1. Visit https://[vercel-url]
2. Click "Chatbot" in navbar
3. Type question: "What is ROS 2?"
4. Click "Ask"
5. Verify answer appears with sources

**Checklist:**
- [ ] Backend health check passes
- [ ] POST /ask returns valid response
- [ ] Frontend loads chatbot page
- [ ] Question/answer flow works
- [ ] Sources are clickable
- [ ] No errors in browser console

---

## Step 7: Setup CI/CD (GitHub Actions)

### Configure GitHub Secrets

Go to GitHub repo → Settings → Secrets and variables → Actions

Add secrets:
```
RAILWAY_TOKEN_PROD=[railway-api-token]
RAILWAY_PROJECT_PROD=[project-id]
VERCEL_TOKEN=[vercel-auth-token]
VERCEL_PROJECT_ID_PROD=[project-id]
VERCEL_ORG_ID=[org-id]
```

**How to get tokens:**

**Railway token:**
```bash
railway login
railway whoami  # Get API token from output
```

**Vercel token:**
```bash
vercel login
vercel whoami  # Get token from settings
```

### Test CI/CD Pipeline

1. Make small change to develop branch
2. Create pull request
3. Watch GitHub Actions run tests
4. Merge to develop to test staging
5. Merge to main to test production

**Checklist:**
- [ ] GitHub Actions workflow shows in Actions tab
- [ ] Backend tests pass
- [ ] Frontend build succeeds
- [ ] Docker image builds
- [ ] Staging deployment succeeds (if configured)
- [ ] Production deployment works

---

## Post-Deployment: Monitoring (24 Hours)

### Hour 1: Critical Checks

- [ ] All endpoints responding (status 200)
- [ ] Error rate < 1%
- [ ] Response times < 3s
- [ ] No database connection errors
- [ ] No OpenAI API errors

### Hour 4: Extended Monitoring

- [ ] Cache hit rate > 20%
- [ ] Database connection pool stable
- [ ] Qdrant search latency < 500ms
- [ ] No repeated errors
- [ ] User base stable

### Day 1: Full Validation

- [ ] User acceptance testing complete
- [ ] Performance metrics stable
- [ ] Database backups working
- [ ] Monitoring alerts configured
- [ ] On-call support ready

### Commands to Monitor

```bash
# View backend logs
railway logs --service backend --tail 100

# View Vercel logs
vercel logs [vercel-url]

# Monitor database
psql $NEON_CONNECTION_STRING -c "SELECT COUNT(*) FROM chat_logs;"

# Check Qdrant collection
curl -H "api-key: $QDRANT_API_KEY" \
  "$QDRANT_URL/collections/documentation_vectors"
```

**Checklist:**
- [ ] No critical errors in logs
- [ ] Response times acceptable
- [ ] Database growing as expected
- [ ] Cache is being used effectively
- [ ] API usage within limits

---

## Rollback Plan

If critical issues occur, follow this procedure:

### Quick Rollback (< 10 minutes)

```bash
# Railway - revert to previous deployment
railway rollback

# Vercel - revert to previous deployment
vercel rollback
```

### Full Rollback (if needed)

1. Update .env variables to point to old backend
2. Redeploy frontend
3. Verify all systems back to previous state
4. Post incident review

**Rollback triggers:**
- [ ] Error rate > 5%
- [ ] Response time > 10s
- [ ] Database connection failures
- [ ] OpenAI API failures
- [ ] Security incident

---

## Post-Deployment Optimization (Week 1)

### Performance Analysis

```bash
# Query slow requests
psql $NEON_CONNECTION_STRING <<SQL
SELECT
  latency_ms,
  COUNT(*) as count
FROM chat_logs
WHERE created_at > NOW() - INTERVAL '24 hours'
GROUP BY latency_ms
ORDER BY latency_ms DESC;
SQL
```

### Optimizations to Consider

- [ ] Adjust RAG_TOP_K if search too slow
- [ ] Increase cache TTL if hit rate low
- [ ] Scale Qdrant cluster if search latency high
- [ ] Review slow database queries
- [ ] Tune rate limiting based on traffic

---

## Deployment Sign-Off

**Backend Status**: ✅ / ❌
- URL: _______________________________
- Health Check: Passing / Failing

**Frontend Status**: ✅ / ❌
- URL: _______________________________
- Chatbot Working: Yes / No

**Database Status**: ✅ / ❌
- Migrations Applied: Yes / No
- Documents Ingested: Yes / No / Count: _____

**Monitoring Status**: ✅ / ❌
- Alerts Configured: Yes / No
- Logs Streaming: Yes / No

**Overall Deployment Status**:

```
[ ] GO - Ready for Production Use
[ ] GO WITH CAUTION - Working but needs monitoring
[ ] NO GO - Critical issues, needs rollback
```

**Approval**:

Deployed By: ________________________  Date: _____________

Verified By: ________________________  Date: _____________

PM/Product Owner: ___________________  Date: _____________

---

## Next Steps After Deployment

1. **Week 1**: Monitor closely, collect user feedback
2. **Week 2**: Deploy Phase 8 enhancements if planned
3. **Week 3+**: Continuous optimization based on metrics
4. **Monthly**: Review performance, plan improvements

---

**Additional Resources:**
- Deployment Guide: See DEPLOYMENT.md
- Operations Runbook: See OPERATIONS.md
- Phase 7 Checklist: See PHASE7_CHECKLIST.md
- GitHub Issues: https://github.com/Nazasif92/Humanoid-Robotic-Book/issues

---

Last Updated: 2024-01-15
Version: 1.0
