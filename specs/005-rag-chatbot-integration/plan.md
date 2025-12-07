# Implementation Plan: RAG Chatbot Integration

**Branch**: `005-rag-chatbot-integration` | **Date**: 2025-12-07 | **Spec**: [specs/005-rag-chatbot-integration/spec.md](spec.md)
**Input**: Feature specification from `/specs/005-rag-chatbot-integration/spec.md`

## Summary

Integrate a Retrieval-Augmented Generation (RAG) chatbot into the Humanoid-Robotics-Book Docusaurus project. The system consists of:
1. **FastAPI backend** (Python 3.11) deployed on Railway with OpenAI GPT-4 integration
2. **Vector DB + Postgres** (Qdrant for embeddings, Neon for metadata)
3. **React frontend** (JSX page in Docusaurus) with conversational UI
4. **Ingestion pipeline** that loads `/docs` markdown files, chunks them (300-500 tokens), embeds with OpenAI, and stores in vector DB

Users ask questions about documentation and receive context-aware answers with cited sources from the knowledge base.

## Technical Context

**Language/Version**: Python 3.11 (backend), JavaScript/JSX (frontend)
**Primary Dependencies**:
  - Backend: FastAPI, Pydantic, OpenAI SDK, Qdrant client, asyncpg (PostgreSQL async)
  - Frontend: React, Docusaurus theme components, fetch API
**Storage**: Neon PostgreSQL (metadata + chat logs), Qdrant (vector embeddings)
**Testing**: pytest (backend unit/integration), Docusaurus build validation (frontend)
**Target Platform**: Linux server (Railway) for backend; Vercel CDN for frontend
**Project Type**: Web application (separate backend API + frontend SPA integration)
**Performance Goals**:
  - p95 response time < 5 seconds (user question → answer display)
  - Ingestion: 1000 chunks/minute
  - Support 10+ concurrent chatbot requests
**Constraints**:
  - CORS restricted to Vercel frontend domain
  - API rate limiting: 20 req/min/IP on /ask endpoint
  - Service-specific timeouts: OpenAI 30s, Qdrant/Neon 5s with exponential backoff (max 3 retries)
**Scale/Scope**:
  - Knowledge base: up to 10,000 documents/chunks
  - Users: Anonymous (no auth in v1)
  - Vector search: <1 second typical query latency

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Constitution Alignment Assessment** (from `.specify/memory/constitution.md`):

| Principle | Requirement | Status | Notes |
|-----------|-------------|--------|-------|
| **I. Accuracy & Correctness** | Code examples must be correct, runnable, produce expected output | ✅ PASS | Ingestion pipeline + backend APIs will include unit tests; frontend UI validated in Docusaurus build |
| **II. Beginner-Friendly Language** | Content uses simple English, no unnecessary jargon | ✅ PASS | Feature is API/backend focused; UI will follow Docusaurus beginner-friendly patterns |
| **III. Consistent Style & Structure** | All chapters follow uniform structure | ✅ PASS | Chatbot page will follow Docusaurus layout conventions; consistent with existing docs |
| **IV. Example-Driven Explanations** | Each concept illustrated with working examples | ✅ PASS | Backend includes example curl commands; frontend shows interaction flows; ingestion demo script provided |
| **V. Quality Review & Fact-Check** | Every feature passes technical accuracy, comprehension, execution, fact-check reviews | ✅ PASS | Implementation plan includes integration tests and validation; code examples are tested |

**Gate Result**: ✅ **PASS** — Feature aligns with all constitution principles. Proceed to Phase 0 research.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
# Web application structure (backend API + Docusaurus frontend integration)

backend/
├── app/
│   ├── __init__.py
│   ├── main.py              # FastAPI app & endpoints
│   ├── rag_pipeline.py      # RAG logic: embedding, search, LLM call
│   ├── qdrant_client.py     # Qdrant vector DB wrapper
│   ├── neon_client.py       # Neon PostgreSQL wrapper (async)
│   ├── models.py            # Pydantic data models
│   └── config.py            # Environment & settings
├── ingest.py                # Standalone ingestion script
├── requirements.txt
├── Dockerfile (for Railway deployment)
├── railway.json
└── tests/
    ├── test_rag_pipeline.py
    ├── test_api_endpoints.py
    └── test_ingestion.py

src/pages/
├── chatbot.jsx              # Docusaurus chatbot page (React)
└── chatbot.module.css       # Styling

.env.example                 # Template for environment variables
```

**Structure Decision**: Web application with separate backend (Python FastAPI) and integrated frontend (React/JSX in Docusaurus). Backend is independently deployable to Railway; frontend is part of Docusaurus build.

## Phase 0: Research & Decisions

### 1. RAG Pattern & OpenAI Integration

**Decision**: Use OpenAI Embedding model (text-embedding-3-small) + GPT-4 Turbo
- **Rationale**: Cost-effective embedding model, best-in-class LLM quality, native API support, proven reliability
- **Alternatives Considered**:
  - Local embeddings (BERT): Lower cost but worse quality, higher infrastructure
  - Anthropic Claude: No native embedding model, would require hybrid setup
  - Open-source LLMs: Lower cost but worse quality, more infrastructure overhead
- **Implementation**: OpenAI SDK with async HTTP calls, error handling with exponential backoff

### 2. Vector Database Selection

**Decision**: Use Qdrant Cloud (managed service)
- **Rationale**: Purpose-built for semantic search, excellent performance (<1s typical query), easy integration, scalable to 10k+ chunks
- **Alternatives Considered**:
  - Pinecone: Easier setup but vendor lock-in, higher cost
  - Weaviate: Good but more complex deployment
  - Chroma: Good for development but not optimized for production scale
- **Implementation**: Docker container locally; Qdrant Cloud for production

### 3. Metadata Storage & Chat Logs

**Decision**: Use Neon PostgreSQL (serverless managed)
- **Rationale**: Reliable, standard SQL, perfect for metadata + chat logs, free tier available, scales on-demand, integrates with async Python via asyncpg
- **Alternatives Considered**:
  - MongoDB: NoSQL but overkill for structured data
  - Firebase/Firestore: Vendor lock-in, more expensive for this scale
  - Direct Qdrant payloads: Would make queries and updates inefficient
- **Implementation**: asyncpg connection pooling, prepared statements for performance

### 4. Document Ingestion Strategy

**Decision**: Recursive markdown parser with Token-based chunking (300-500 tokens)
- **Rationale**: Preserves document structure, respects semantic boundaries, optimal for RAG (not too small, not too large)
- **Alternatives Considered**:
  - Fixed paragraph chunking: Loses context across boundaries
  - Sentence-level: Too granular, poor retrieval
  - Whole documents: Too large, poor precision
- **Implementation**: Use `tiktoken` for accurate token counting, preserve metadata (title, section, URL) per chunk

### 5. Frontend Integration

**Decision**: React JSX page at `/chatbot` integrated into Docusaurus navbar
- **Rationale**: Native to existing architecture, reuses Docusaurus styling/theme, no additional SPA overhead
- **Alternatives Considered**:
  - Iframe embedded bot: Isolation but poor UX, no theme consistency
  - Floating widget: Less discoverable, conflicts with content
  - Separate SPA: Duplicates tooling, complicates deployment
- **Implementation**: Create `src/pages/chatbot.jsx` with custom styling

### 6. Backend Deployment

**Decision**: Use Railway with Docker + environment variables
- **Rationale**: Simple Git integration, easy scaling, competitive pricing, supports scheduled jobs (for ingestion)
- **Alternatives Considered**:
  - Render: Similar, but Railway has better async support for Python
  - Heroku: Sunsetting, expensive
  - AWS Lambda: Overkill, cold start issues for RAG
- **Implementation**: Dockerfile with uvicorn, railway.json for config, GitHub Actions for auto-deploy

### 7. Rate Limiting & Timeout Policies

**Decision**: Implement middleware-based rate limiting + service-specific timeouts
- **Rationale**: Prevents abuse, handles transient failures gracefully, balances user experience with reliability
- **Alternatives Considered**:
  - No rate limiting: Risk of abuse/cost overruns
  - Redis-based distributed limiting: Overkill for initial scale
- **Implementation**: SlowAPI library for /ask endpoint (20 req/min/IP), exponential backoff for service calls

---

## Phase 1: Design & API Contracts

### Data Model

See: [`data-model.md`](data-model.md)

**Key Entities:**

1. **documents** (PostgreSQL)
   - id: SERIAL PRIMARY KEY
   - title: TEXT (extracted from frontmatter)
   - section: TEXT (doc path, e.g., "chapters/ros2-nervous-system")
   - url: TEXT (Docusaurus URL, e.g., "/docs/chapters/ros2-nervous-system")
   - content: TEXT (full markdown)
   - created_at: TIMESTAMP DEFAULT NOW()
   - updated_at: TIMESTAMP

2. **chunks** (Qdrant vectors + PostgreSQL metadata)
   - id: SERIAL PRIMARY KEY (PostgreSQL)
   - doc_id: INTEGER FK → documents.id
   - chunk_text: TEXT (300-500 tokens)
   - chunk_index: INTEGER (position in document)
   - embedding: VECTOR[1536] (in Qdrant; stored as float32 array)
   - section: TEXT (inherited from document)
   - created_at: TIMESTAMP

3. **chat_logs** (PostgreSQL)
   - id: SERIAL PRIMARY KEY
   - question: TEXT
   - answer: TEXT
   - sources: JSONB array ([{title, section, url}, ...])
   - created_at: TIMESTAMP DEFAULT NOW()

### API Contracts

See: [`contracts/api.openapi.yaml`](contracts/api.openapi.yaml)

**Key Endpoints:**

1. **POST /ask**
   - Request: `{ "question": string, "selected_text"?: string }`
   - Response: `{ "answer": string, "sources": [{title, section, url}, ...], "status": "success"|"error" }`
   - Timeout: 5 seconds (client) + 30 seconds (server with retries)

2. **GET /health**
   - Response: `{ "status": "ok"|"degraded", "checks": { "qdrant": bool, "neon": bool, "openai": bool } }`
   - Timeout: 2 seconds

3. **POST /ingest** (internal, for scheduled jobs)
   - Request: `{ "docs_path": "/path/to/docs", "batch_size"?: 100 }`
   - Response: `{ "chunks_created": int, "duration_seconds": float }`

### Quickstart

See: [`quickstart.md`](quickstart.md)

---

## Next Phase

→ Run `/sp.tasks` to generate implementation tasks with test cases and acceptance criteria.
