# Quick Start: Deploy RAG Chatbot to Production

**Estimated Time**: 45 minutes
**Prerequisites**: All accounts created (see DEPLOYMENT_CHECKLIST.md)

---

## Part 1: Gather Your Credentials (5 min)

Before starting, collect these values:

```bash
# OpenAI
OPENAI_API_KEY="sk-..."

# Neon PostgreSQL
NEON_CONNECTION_STRING="postgresql://user:password@host/dbname?sslmode=require"

# Qdrant Cloud
QDRANT_URL="https://your-cluster.qdrant.io"
QDRANT_API_KEY="..."

# Railway
RAILWAY_PROJECT_ID="..."
RAILWAY_API_TOKEN="..."

# Vercel
VERCEL_PROJECT_ID="..."
VERCEL_ORG_ID="..."
VERCEL_AUTH_TOKEN="..."

# Final URLs (from Railway/Vercel after first deploy)
RAILWAY_URL="https://your-service.railway.app"
VERCEL_URL="https://your-domain.vercel.app"
```

Save these in a password manager or secure location.

---

## Part 2: Deploy Backend (10 min)

### Option A: Using Railway Dashboard (Easiest)

1. **Go to Railway**: https://railway.app
2. **Create Project** (if needed)
3. **Connect GitHub**: Select `Humanoid-Robotic-Book` repo
4. **Select Branch**: `005-rag-chatbot-integration`
5. **Configure Environment**:
   - Click "Settings" in your project
   - Go to "Environment"
   - Add all variables from DEPLOYMENT_CHECKLIST.md
6. **Deploy**: Click "Deploy" button
7. **Wait**: Build completes (3-5 min)

### Option B: Using Railway CLI

```bash
# Install
npm install -g @railway/cli

# Login
railway login

# Navigate to project
cd /path/to/project

# Link to Railway project
railway link --project $RAILWAY_PROJECT_ID

# Add environment variables
railway variables set OPENAI_API_KEY=$OPENAI_API_KEY
railway variables set NEON_CONNECTION_STRING=$NEON_CONNECTION_STRING
railway variables set QDRANT_URL=$QDRANT_URL
railway variables set QDRANT_API_KEY=$QDRANT_API_KEY
railway variables set FRONTEND_URL=$VERCEL_URL

# Deploy
railway deploy --service backend

# Watch deployment
railway logs --follow
```

### Verify Backend Works

```bash
# Get your Railway URL
RAILWAY_URL=$(railway status | grep "URL" | awk '{print $NF}')
echo $RAILWAY_URL

# Test health endpoint
curl $RAILWAY_URL/health

# Expected: {"status":"ok",...}
```

✅ Backend deployed successfully!

---

## Part 3: Initialize Database (5 min)

### Run Migrations

```bash
# Set your connection string
export NEON_CONNECTION_STRING="postgresql://..."

# Run migrations
python backend/migrate.py --up

# Verify
python backend/migrate.py --status
```

**Output should show:**
```
Applied (1):
  ✓ 001_initial_schema
```

### Ingest Documentation

```bash
# Make sure you're in backend directory
cd backend

# Run ingestion (this takes 2-3 minutes for 40+ documents)
python ingest.py --init-db --docs-path ../docs

# Watch progress
# Status: SUCCESS
# Documents processed: 42
# Chunks created: 1247
```

✅ Database initialized and documentation ingested!

---

## Part 4: Deploy Frontend (10 min)

### Option A: Using Vercel Dashboard (Easiest)

1. **Go to Vercel**: https://vercel.com
2. **Create Project** (if needed)
3. **Import Repository**: Select `Humanoid-Robotic-Book`
4. **Select Branch**: `005-rag-chatbot-integration`
5. **Configure Environment**:
   - Project Settings → Environment Variables
   - Add `NEXT_PUBLIC_BACKEND_URL=$RAILWAY_URL`
   - Add `NEXT_PUBLIC_FRONTEND_URL=$VERCEL_URL`
6. **Deploy**: Click "Deploy" button
7. **Wait**: Build completes (2-3 min)

### Option B: Using Vercel CLI

```bash
# Install
npm install -g vercel

# Deploy to production
vercel --prod

# Follow interactive prompts:
# - Use existing project? Yes
# - Which project? humanoid-robotic-book
# - Override settings? Yes (if needed)

# Get your Vercel URL
VERCEL_URL=$(vercel ls | grep "humanoid" | awk '{print $NF}')
```

### Verify Frontend Works

```bash
# Visit your site
open https://humanoid-robotic-book.vercel.app

# Or test via CLI
curl https://humanoid-robotic-book.vercel.app/chatbot | head -20
```

✅ Frontend deployed successfully!

---

## Part 5: Test End-to-End (5 min)

### Test Backend API

```bash
# Test health
curl https://$RAILWAY_URL/health

# Test RAG endpoint
curl -X POST https://$RAILWAY_URL/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "selected_text": null
  }'

# You should get a response like:
# {
#   "answer": "ROS 2 is a middleware...",
#   "sources": [
#     {
#       "title": "ROS 2 Basics",
#       "section": "Introduction",
#       "url": "/docs/ros2-basics"
#     }
#   ],
#   "status": "success",
#   "latency_ms": 1234
# }
```

### Test Frontend Interaction

1. **Visit**: https://humanoid-robotic-book.vercel.app
2. **Click**: "Chatbot" link in navbar
3. **Type**: "What is ROS 2?" in the question box
4. **Click**: "Ask" button
5. **Verify**:
   - ✅ Answer appears (not empty)
   - ✅ Sources are listed with clickable links
   - ✅ Response time shown
   - ✅ No red error box

✅ Everything works end-to-end!

---

## Part 6: Setup CI/CD for Future Deployments (5 min)

### Create GitHub Deploy Tokens

```bash
# You'll need these secrets in GitHub
# Go to repo Settings → Secrets and variables → Actions

# Create these secrets:
RAILWAY_TOKEN_PROD="..."           # From: railway auth-token
RAILWAY_PROJECT_PROD="..."          # From: railway status
VERCEL_TOKEN="..."                  # From: vercel tokens create
VERCEL_PROJECT_ID_PROD="..."       # From: Vercel dashboard
VERCEL_ORG_ID="..."                # From: Vercel dashboard
```

### How to Get Tokens

**Railway token:**
```bash
railway auth-token
# Copy the long token shown
```

**Vercel token:**
```bash
# Via Vercel dashboard:
# Settings → Tokens → Create
```

### Automatic Deployments Now Enabled

```bash
# Now any push to:
# - develop branch → Auto-deploys to staging
# - main branch → Auto-deploys to production

# Test by creating a small change:
echo "# Test change" >> README.md
git add README.md
git commit -m "Test CI/CD pipeline"
git push origin 005-rag-chatbot-integration

# Watch GitHub Actions → Actions tab
```

✅ CI/CD pipeline ready!

---

## Part 7: Monitor Your Deployment (Ongoing)

### Daily Health Checks

```bash
# Every morning, run:
curl https://your-backend-url/health

# Should return: "status":"ok"
```

### Watch the Logs

```bash
# View backend logs
railway logs --follow

# View frontend logs (Vercel)
vercel logs https://your-domain.vercel.app
```

### Monitor Key Metrics

```bash
# Check database is growing
psql $NEON_CONNECTION_STRING -c "
  SELECT
    'chats' as table_name,
    COUNT(*) as rows,
    ROUND(pg_total_relation_size('chat_logs')/1024/1024, 2) as size_mb
  FROM chat_logs;
"

# Check Qdrant vectors
curl -H "api-key: $QDRANT_API_KEY" \
  "$QDRANT_URL/collections/documentation_vectors" | jq '.result.points_count'
```

---

## Troubleshooting

### "502 Bad Gateway"
```bash
# Check backend health
curl https://your-railway-url/health

# View logs
railway logs

# Most likely: Database or API key issue
# Fix: Check NEON_CONNECTION_STRING and OPENAI_API_KEY
```

### "CORS Error in Browser"
```bash
# Check FRONTEND_URL in Railway env matches Vercel URL
railway variables | grep FRONTEND_URL

# Should show your actual Vercel domain
# Fix: Update FRONTEND_URL if wrong, redeploy
```

### "No results from chatbot"
```bash
# Check documents were ingested
psql $NEON_CONNECTION_STRING -c "SELECT COUNT(*) FROM chunks;"

# Should show > 100 chunks

# If 0, run ingestion again:
python backend/ingest.py --init-db --docs-path ../docs
```

### "Slow responses (> 5s)"
```bash
# Check OpenAI status
curl https://status.openai.com/api/v2/status.json

# If OpenAI is slow, nothing you can do
# Otherwise, try reducing RAG_TOP_K:
railway variables set RAG_TOP_K=2
railway redeploy
```

---

## Success Checklist

- ✅ Backend deployed to Railway
- ✅ Database migrations applied
- ✅ Documents ingested (> 100 chunks)
- ✅ Frontend deployed to Vercel
- ✅ Chatbot page loads and works
- ✅ Health endpoint returns OK
- ✅ CI/CD pipeline configured
- ✅ Monitoring logs visible
- ✅ Users can ask questions and get answers

**You're live in production! 🎉**

---

## Next Steps

1. **Monitor** (first 24 hours): Check logs every hour
2. **Collect feedback**: Ask users for issues
3. **Optimize** (week 2): Fine-tune RAG_TOP_K, cache settings
4. **Enhance** (week 3+): Add features, improve answers
5. **Scale** (as needed): Upgrade Qdrant/Neon if traffic grows

---

## Documentation Reference

| Document | Purpose |
|----------|---------|
| DEPLOYMENT.md | Detailed technical guide |
| OPERATIONS.md | Daily/incident procedures |
| DEPLOYMENT_CHECKLIST.md | Step-by-step checklist |
| PHASE7_CHECKLIST.md | Pre-deployment verification |
| QUICKSTART_DEPLOYMENT.md | This file (quick guide) |

---

## Support

**If something breaks:**

1. Check DEPLOYMENT_CHECKLIST.md section "Troubleshooting"
2. Review OPERATIONS.md for incident procedures
3. Check GitHub Issues: https://github.com/Nazasif92/Humanoid-Robotic-Book/issues
4. Review logs: `railway logs` or Vercel dashboard

---

**Estimated total time**: 45 minutes from start to live production

**Questions?** Start with DEPLOYMENT.md for detailed information.

---

Last Updated: 2024-01-15
Version: 1.0
