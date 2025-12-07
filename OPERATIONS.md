# Operations Runbook

Quick reference guide for common operational tasks on the RAG Chatbot system.

## Table of Contents
- [Daily Checks](#daily-checks)
- [Incident Response](#incident-response)
- [Database Maintenance](#database-maintenance)
- [Performance Tuning](#performance-tuning)
- [Scaling Operations](#scaling-operations)

---

## Daily Checks

### Morning Health Check (5 min)

```bash
#!/bin/bash
# Check backend health
BACKEND_URL="https://your-railway-url"

# 1. API health
echo "Backend health:"
curl -s $BACKEND_URL/health | jq '.status'

# 2. Database connectivity
echo "Database status:"
psql $NEON_CONNECTION_STRING -c "SELECT NOW();"

# 3. Vector DB connectivity
echo "Qdrant status:"
curl -s $QDRANT_URL/health | jq '.status'
```

**Acceptance criteria:**
- ✓ Backend returns status: "ok"
- ✓ Database query returns current timestamp
- ✓ Qdrant returns 200 OK

### Weekly Report (15 min)

```bash
#!/bin/bash
# Weekly metrics

echo "=== WEEKLY REPORT ==="
echo ""

# Chat statistics
echo "Chat Activity:"
psql $NEON_CONNECTION_STRING <<SQL
SELECT
  DATE(created_at) as date,
  COUNT(*) as total_chats,
  AVG(latency_ms) as avg_latency_ms,
  MAX(latency_ms) as max_latency_ms
FROM chat_logs
WHERE created_at > NOW() - INTERVAL '7 days'
GROUP BY DATE(created_at)
ORDER BY date DESC;
SQL

# Error summary
echo ""
echo "Errors (last 7 days):"
psql $NEON_CONNECTION_STRING <<SQL
SELECT
  error,
  COUNT(*) as count
FROM chat_logs
WHERE error IS NOT NULL
  AND created_at > NOW() - INTERVAL '7 days'
GROUP BY error
ORDER BY count DESC;
SQL

# Database size
echo ""
echo "Database usage:"
psql $NEON_CONNECTION_STRING <<SQL
SELECT
  datname as database,
  pg_size_pretty(pg_database_size(datname)) as size
FROM pg_database
WHERE datname = current_database();
SQL

# Vector index stats
echo ""
echo "Vector collection stats:"
curl -s "$QDRANT_URL/collections/documentation_vectors" \
  -H "api-key: $QDRANT_API_KEY" | jq '.result.points_count'
```

---

## Incident Response

### Issue: API Returns 500 Error

**Detection:** Automated alerts or user reports

**Investigation (2 min):**
```bash
# 1. Check recent logs
railway logs --tail 50

# 2. Verify backend is running
curl https://your-railway-url/health

# 3. Check database connectivity
psql $NEON_CONNECTION_STRING -c "SELECT 1;"

# 4. Verify Qdrant is accessible
curl $QDRANT_URL/health
```

**Common causes & fixes:**
| Issue | Symptom | Fix |
|-------|---------|-----|
| Database down | Connection refused | Check Neon console; restart DB |
| Qdrant down | "Unable to reach Qdrant" | Check Qdrant Cloud console |
| API key expired | "Invalid API key" | Rotate keys in Railway settings |
| OOM (Out of Memory) | Slow/hanging requests | Restart service; check for leaks |

**Resolution example:**
```bash
# If Neon is having issues, check status
# Then restart Railway service:
railway restart

# Monitor restart:
railway logs --follow
```

### Issue: Slow Responses (> 5 seconds)

**Detection:** Monitor API latency dashboard

**Investigation (5 min):**
```bash
# 1. Check OpenAI API status
curl https://status.openai.com/api/v2/status.json | jq '.'

# 2. Check Qdrant latency
# (Via Qdrant Cloud console → Metrics)

# 3. Check database slow queries
psql $NEON_CONNECTION_STRING <<SQL
SELECT
  query,
  calls,
  mean_exec_time,
  max_exec_time
FROM pg_stat_statements
WHERE mean_exec_time > 100
ORDER BY mean_exec_time DESC
LIMIT 10;
SQL
```

**Solutions:**
```bash
# Reduce RAG_TOP_K (fewer vectors to fetch)
# Railway → Environment → RAG_TOP_K=2

# Increase Qdrant cluster resources
# Qdrant Cloud Console → Cluster Settings

# Optimize database indexes
python backend/migrate.py --status
# Re-run migrations if needed
```

### Issue: Out of Rate Limits

**Detection:** Users see "429 Too Many Requests"

**Temporary fix (1 min):**
```bash
# Increase rate limit temporarily
# Railway → Environment → RATE_LIMIT_REQUESTS=50
railway redeploy
```

**Long-term solution:**
```bash
# Analyze traffic patterns
psql $NEON_CONNECTION_STRING <<SQL
SELECT
  DATE_TRUNC('hour', created_at) as hour,
  COUNT(*) as requests_per_hour
FROM chat_logs
WHERE created_at > NOW() - INTERVAL '1 day'
GROUP BY hour
ORDER BY requests_per_hour DESC;
SQL

# If sustained high traffic:
# - Scale Qdrant to higher tier
# - Enable caching for common questions
# - Increase Railway instance size
```

---

## Database Maintenance

### Backup & Restore

**Neon automatic backups:**
```bash
# Backups are automatic
# View in Neon Console → Backups
# Retention: 7 days hourly, 30 days daily
```

**Manual backup before major changes:**
```bash
# Backup
pg_dump -h $(echo $NEON_CONNECTION_STRING | grep -oP '(?<=@)[^/]*') \
        -U $(echo $NEON_CONNECTION_STRING | grep -oP '(?<=//).*?(?=:)') \
        > backup_$(date +%Y%m%d_%H%M%S).sql

# Verify backup
file backup_*.sql
wc -l backup_*.sql
```

**Restore from backup:**
```bash
# WARNING: This overwrites data
psql $NEON_CONNECTION_STRING < backup_20240115_120000.sql

# Verify
psql $NEON_CONNECTION_STRING -c "SELECT COUNT(*) FROM chat_logs;"
```

### Cleanup Old Data

**Archive old chat logs (6+ months):**
```bash
# Export to CSV for archival
psql $NEON_CONNECTION_STRING <<SQL
COPY (
  SELECT * FROM chat_logs
  WHERE created_at < NOW() - INTERVAL '6 months'
) TO STDOUT WITH CSV HEADER;
SQL > archive_$(date +%Y%m).csv

# Delete archived data
psql $NEON_CONNECTION_STRING <<SQL
DELETE FROM chat_logs
WHERE created_at < NOW() - INTERVAL '6 months';
SQL

# Reindex to reclaim space
VACUUM ANALYZE chat_logs;
```

### Monitor Connection Pool

```bash
# Current active connections
psql $NEON_CONNECTION_STRING <<SQL
SELECT
  usename,
  application_name,
  state,
  COUNT(*) as count
FROM pg_stat_activity
GROUP BY usename, application_name, state;
SQL

# If connection pool exhausted:
# 1. Increase NEON_POOL_MAX_SIZE
# 2. Check for connection leaks in code
# 3. Restart affected services
```

---

## Performance Tuning

### Database Query Optimization

**Identify slow queries:**
```bash
psql $NEON_CONNECTION_STRING <<SQL
SELECT
  query,
  calls,
  mean_exec_time,
  max_exec_time
FROM pg_stat_statements
WHERE mean_exec_time > 50
ORDER BY mean_exec_time DESC
LIMIT 5;
SQL
```

**Add missing indexes:**
```sql
-- Most common index additions
CREATE INDEX idx_chat_logs_latency ON chat_logs(latency_ms);
CREATE INDEX idx_chunks_tokens ON chunks(token_count);

-- Analyze impact
ANALYZE;
```

### Vector Search Optimization

**Tune similarity threshold:**
```bash
# Current setting
echo $RAG_MIN_SIMILARITY  # 0.5

# Increase for faster but stricter search
# Railway → Environment → RAG_MIN_SIMILARITY=0.6
```

**Adjust top-K results:**
```bash
# Fetch fewer vectors (faster)
# Railway → Environment → RAG_TOP_K=2  # default 3

# vs. fetch more (better coverage)
# RAG_TOP_K=5
```

**Monitor vector search performance:**
```bash
# Via Qdrant Cloud Console → Metrics
# Look for:
# - Search latency p95 < 500ms
# - Search success rate > 99%
```

---

## Scaling Operations

### Vertical Scaling (Bigger Instance)

**When:** Single request throughput is bottleneck

**Steps:**
```bash
# 1. In Railway console: Deployment → Instance Type
#    Change from Standard to Large

# 2. Monitor during scaling
railway logs --follow

# 3. Run performance test after
curl -X POST https://your-railway-url/ask \
  -H "Content-Type: application/json" \
  -d '{"question":"What is ROS 2?"}' \
  -w "Response time: %{time_total}s\n"
```

### Horizontal Scaling (Multiple Instances)

**When:** Multiple concurrent requests need load distribution

**Steps:**
```bash
# 1. Enable multiple replicas in Railway
# Railway → Deployment → "Number of Replicas" = 3

# 2. Setup load balancer (Railway does this automatically)

# 3. Test load distribution
for i in {1..10}; do
  curl -X POST https://your-railway-url/ask \
    -H "Content-Type: application/json" \
    -d "{\"question\":\"Test $i\"}" &
done
wait

# 4. Monitor instance logs
railway logs --instance 0
railway logs --instance 1
railway logs --instance 2
```

### Database Scaling

**For increased connections:**
```bash
# Neon Console → Project Settings → Compute
# Increase "Max connections" setting
# Default: 100, Max: 10000

# Also update application pool:
# Railway → Environment → NEON_POOL_MAX_SIZE=30
```

**For increased data volume:**
```bash
# Neon Console → Billing → Plan upgrade
# Standard → Professional (higher storage, connections, support)

# Monitor storage:
psql $NEON_CONNECTION_STRING -c "
SELECT pg_size_pretty(pg_database_size(current_database()));"
```

### Qdrant Scaling

**For increased vector volume:**
```bash
# Qdrant Cloud Console → Cluster
# Upgrade cluster tier when:
# - Points count > 80% of limit
# - Search latency p95 > 1000ms
# - During peak load, CPU > 80%
```

---

## Troubleshooting Reference

| Symptom | Likely Cause | Quick Fix |
|---------|--------------|-----------|
| "502 Bad Gateway" | Backend crash | Check logs: `railway logs` |
| "429 Rate Limited" | Too many requests | Increase limit in Railway env |
| "CORS error" | Frontend URL mismatch | Verify FRONTEND_URL in Railway |
| "> 5s response time" | OpenAI or Qdrant slow | Reduce RAG_TOP_K or check status |
| "No results found" | No relevant vectors | Check ingestion: `psql ... SELECT COUNT(*) FROM chunks;` |
| "Connection refused" | Database offline | Check Neon console status |
| "Invalid API key" | Expired/rotated key | Regenerate and update Railway |

---

## Escalation

**When to escalate:**

1. **Critical (Page on-call):**
   - API completely down (0% success rate)
   - Database data corruption
   - Security breach detected

2. **High (Within 1 hour):**
   - API intermittently failing
   - Database performance degradation
   - High error rates (> 5%)

3. **Medium (Within 4 hours):**
   - Slow responses (> 5s)
   - Resource exhaustion warnings
   - Minor data inconsistency

**Escalation contacts:**
- Platform Ops: [ops-team contact]
- On-Call Engineer: [pagerduty-link]
- Database Admin: [dba-team contact]

---

**Last Updated:** 2024-01-15
**Version:** 1.0
