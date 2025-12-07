# Implementation Tasks: RAG Chatbot Integration

**Feature**: RAG Chatbot Integration | **Branch**: `005-rag-chatbot-integration` | **Date**: 2025-12-07

**Total Tasks**: 45 | **Phases**: 6 | **User Stories**: 3 | **MVP Scope**: Phase 1-3 (US1 only)

---

## Task Numbering & Format

All tasks follow strict format: `- [ ] [ID] [P?] [Story?] Description with file path`

- **ID**: Task identifier (T001, T002, etc.)
- **[P]**: Parallelizable (tasks can run in parallel)
- **[Story]**: User story label ([US1], [US2], [US3])
- **Description**: Clear action with exact file path
- **File paths**: Absolute within project root

---

## Implementation Strategy

### MVP (Minimum Viable Product)
**Phases 1-3**: Core question-answering functionality (User Story 1)
- Backend API ready for questions
- Knowledge base indexed
- Frontend chatbot UI working
- ~2-3 weeks of development

### Phase 2 (Nice-to-Have)
**Phase 4**: Selected text context feature (User Story 2)
- ~1 week additional development

### Phase 3 (Optional)
**Phase 5**: Chat history (User Story 3)
- ~1 week additional development

---

## Execution Plan

### Dependency Graph

```
Phase 1: Setup
    ↓
Phase 2: Infrastructure & Database
    ↓
Phase 3: Backend Core (US1) ← Start here for MVP
Phase 4: Frontend Core (US1)
    ↓↓ (parallel: Phase 5 & 6 after Phase 3)
Phase 5: US2 Enhancements (selected text)
Phase 6: US3 Enhancements (chat history)
    ↓
Phase 7: Polish & Deployment
```

### Parallel Opportunities

**Phase 1 Setup**: All tasks parallelizable
**Phase 2**: Database tasks can run in parallel
**Phase 3**: RAG pipeline tasks partially parallelizable (after models defined)
**Phase 4**: Frontend and backend can develop in parallel (contract-driven)

---

## PHASE 1: PROJECT SETUP

**Goal**: Initialize project structure, dependencies, and configuration

### Setup Tasks

- [ ] T001 Create backend project structure with directories: backend/app/, backend/tests/, backend/ingest.py
- [ ] T002 [P] Create frontend pages directory: src/pages/
- [ ] T003 Create requirements.txt with FastAPI, Pydantic, OpenAI, Qdrant, asyncpg, pytest, python-dotenv
- [ ] T004 [P] Create .env.example with template variables (OPENAI_API_KEY, NEON_CONNECTION_STRING, QDRANT_URL, etc.)
- [ ] T005 [P] Create backend/app/__init__.py (empty initialization file)
- [ ] T006 Create backend/app/config.py with Pydantic Settings for environment variables
- [ ] T007 Create Dockerfile for backend with Python 3.11 base image, requirements install, uvicorn entrypoint
- [ ] T008 [P] Create railway.json with build and start commands for Railway deployment
- [ ] T009 Create backend/tests/ directory structure with __init__.py and conftest.py

---

## PHASE 2: INFRASTRUCTURE & DATABASE

**Goal**: Set up database connections, ORM models, and client wrappers

### Database Schema & Models

- [ ] T010 Create Neon PostgreSQL tables using asyncpg: documents, chunks, chat_logs (see data-model.md)
- [ ] T011 [P] Create Qdrant collection 'humanoid_docs' with vector size 1536, cosine distance metric
- [ ] T012 [P] Create backend/app/models.py with Pydantic models: Document, Chunk, ChatLog, AskRequest, AskResponse, Source
- [ ] T013 [P] Define Pydantic validators for models (question length <5000, chunk_text length validation)

### Client Wrappers

- [ ] T014 Create backend/app/neon_client.py with async PostgreSQL connection pool and CRUD operations
  - Methods: init_db(), create_document(), create_chunk(), log_chat()
- [ ] T015 [P] Create backend/app/qdrant_client.py with Qdrant collection management and search
  - Methods: init_collection(), upsert_vectors(), search_similar()

### Configuration & Environment

- [ ] T016 [P] Implement environment variable validation in backend/app/config.py (raise error if missing required keys)
- [ ] T017 Update backend/app/__init__.py to export config and models

---

## PHASE 3: BACKEND CORE - USER STORY 1

**Goal**: Implement core RAG pipeline and API endpoints for question-answering

### RAG Pipeline Implementation

- [ ] T018 [US1] Create backend/app/rag_pipeline.py with class RAGPipeline
  - Methods: embed_query(), search_knowledge_base(), retrieve_chunks_from_db(), call_llm()
- [ ] T019 [US1] Implement query embedding using OpenAI API with error handling and timeout (30s)
- [ ] T020 [US1] Implement Qdrant semantic search with top-K retrieval (K=3-5 configurable)
- [ ] T021 [US1] Implement Neon chunk metadata fetch by vector IDs
- [ ] T022 [US1] Implement OpenAI GPT-4 call with system prompt for RAG (instruct model to cite sources, avoid hallucination)
- [ ] T023 [US1] Implement exponential backoff retry logic for OpenAI, Qdrant, Neon timeouts (3 retries max)
- [ ] T024 [US1] Add response source extraction and formatting (title, section, url, chunk_text)

### API Endpoints

- [ ] T025 [US1] Create backend/app/main.py with FastAPI application setup
- [ ] T026 [US1] [P] Implement POST /ask endpoint: request validation, RAG call, response formatting, error handling
- [ ] T027 [US1] [P] Implement GET /health endpoint: check Neon, Qdrant, OpenAI connectivity, return status
- [ ] T028 [US1] Implement CORS middleware: allow Vercel frontend domain, restrict to specified origins
- [ ] T029 [US1] Implement rate limiting on /ask: 20 requests/minute per IP using SlowAPI
- [ ] T030 [US1] [P] Implement POST /ingest endpoint: accept docs_path, call ingestion pipeline, return chunk count

### Logging & Monitoring

- [ ] T031 [US1] Implement structured logging with Python logging module (INFO, ERROR levels)
- [ ] T032 [US1] Add latency tracking in RAGPipeline: measure embedding, search, LLM call times
- [ ] T033 [US1] Update chat_logs table insert to include latency_ms, error field

---

## PHASE 4: INGESTION PIPELINE & FRONTEND - USER STORY 1

**Goal**: Create document ingestion system and chatbot UI page

### Document Ingestion

- [ ] T034 Create backend/ingest.py as standalone script
- [ ] T035 [P] Implement markdown file discovery: recursive scan of /docs folder, extract .md files
- [ ] T036 [P] Implement frontmatter parser: extract title from markdown YAML front matter
- [ ] T037 Implement markdown-to-plain-text converter: strip markdown syntax, preserve content
- [ ] T038 [P] Implement token-based document chunking using tiktoken: 300-500 token chunks, preserve structure
- [ ] T039 Implement chunk embedding: call OpenAI API for each chunk, handle batch processing
- [ ] T040 Implement Neon storage: insert documents, chunks with metadata (title, section, url, chunk_index)
- [ ] T041 Implement Qdrant upsert: store vectors with doc_id, chunk_index payloads
- [ ] T042 Add --init-db flag to ingest.py to create tables
- [ ] T043 Add error handling and logging to ingestion script (skip files on error, report summary)

### Frontend Chatbot Page

- [ ] T044 [US1] Create src/pages/chatbot.jsx with React component for chatbot UI
- [ ] T045 [US1] [P] Implement question input field, submit button, loading indicator
- [ ] T046 [US1] [P] Implement answer display area with markdown rendering
- [ ] T047 [US1] Implement sources display: clickable links to original docs (url from response)
- [ ] T048 [US1] Implement error handling: show error message if backend unavailable
- [ ] T049 [US1] Implement response latency display (optional, from latency_ms response field)
- [ ] T050 [US1] Create src/pages/chatbot.module.css with styling for chatbot UI
- [ ] T051 [US1] [P] Update docusaurus.config.js: add Chatbot link to navbar (position: 'right')

### Testing - User Story 1

- [ ] T052 [US1] Create backend/tests/test_rag_pipeline.py: unit tests for embedding, search, source extraction
- [ ] T053 [US1] Create backend/tests/test_api_endpoints.py: test /ask, /health with mocked dependencies
- [ ] T054 [US1] Create backend/tests/test_ingestion.py: test markdown parsing, chunking, embedding storage
- [ ] T055 [US1] Add integration test: end-to-end question → answer flow with real Neon/Qdrant

---

## PHASE 5: USER STORY 2 - SELECTED TEXT CONTEXT

**Goal**: Enable users to provide selected text as context for answers

**Dependency**: Requires Phase 3-4 complete (core /ask endpoint working)

### Backend Enhancement

- [ ] T056 [US2] Update backend/app/rag_pipeline.py RAGPipeline.call_llm(): include selected_text in system prompt
- [ ] T057 [US2] Modify prompt engineering: instruct model to use selected_text as primary context if provided
- [ ] T058 [US2] Update POST /ask response: include source reference for selected_text if provided

### Frontend Enhancement

- [ ] T059 [US2] [P] Update src/pages/chatbot.jsx: add optional "selected_text" textarea input
- [ ] T060 [US2] [P] Implement JavaScript function to detect text selection on page: copy to context field
- [ ] T061 [US2] Implement "Use Selection" button: populate context from user's highlighted text
- [ ] T062 [US2] Update fetch call to /ask: include selected_text in JSON body

### Testing - User Story 2

- [ ] T063 [US2] Create test: verify /ask with selected_text produces answer referencing context
- [ ] T064 [US2] Create test: verify selected_text appears in sources array (as a source type)
- [ ] T065 [US2] Create test: verify /ask works without selected_text (graceful fallback)

---

## PHASE 6: USER STORY 3 - CHAT HISTORY

**Goal**: Allow users to view and restore previous questions and answers

**Dependency**: Requires Phase 3-4 complete (chat_logs table populated)

### Backend Enhancement

- [ ] T066 [US3] Implement POST /chat-history endpoint: return paginated chat_logs (limit 50, sorted DESC by created_at)
- [ ] T067 [US3] [P] Implement GET /chat-history/:id endpoint: return full chat_log entry by ID

### Frontend Enhancement

- [ ] T068 [US3] Update src/pages/chatbot.jsx: add "History" panel/sidebar (collapsible)
- [ ] T069 [US3] Implement fetch call to /chat-history: load chat logs on component mount
- [ ] T070 [US3] [P] Implement history display: render list of previous questions with timestamps
- [ ] T071 [US3] Implement click handler: restore previous answer to chat display on history item click
- [ ] T072 [US3] Add local browser storage (sessionStorage): save recent chats to session

### Testing - User Story 3

- [ ] T073 [US3] Create test: verify /chat-history returns paginated results
- [ ] T074 [US3] Create test: verify chat_logs entries are created after /ask calls
- [ ] T075 [US3] Create test: verify history panel renders correctly with multiple chats

---

## PHASE 7: POLISH & DEPLOYMENT

**Goal**: Final testing, documentation, and production deployment

### Quality Assurance

- [ ] T076 Run full backend test suite: pytest tests/ -v (target: >85% coverage)
- [ ] T077 [P] Run Docusaurus build validation: npm run build (must pass without errors)
- [ ] T078 Performance test: measure /ask latency with load testing (target: p95 < 5s for 10 concurrent requests)
- [ ] T079 CORS test: verify frontend can call backend without CORS errors
- [ ] T080 Rate limiting test: verify 20 req/min limit is enforced on /ask endpoint

### Documentation

- [ ] T081 Add README.md to backend/ with setup, testing, deployment instructions
- [ ] T082 [P] Create API_DOCUMENTATION.md: endpoint descriptions, example requests/responses, error codes
- [ ] T083 Update quickstart.md: add test execution steps and expected outputs
- [ ] T084 [P] Add code comments: docstrings for all public functions in backend/app/

### Deployment Configuration

- [ ] T085 Create GitHub Actions workflow (.github/workflows/backend-tests.yml) for pytest on PR
- [ ] T086 [P] Configure Neon connection pooling for production (max 20 connections)
- [ ] T087 [P] Set up Qdrant backup schedule in Qdrant Cloud dashboard
- [ ] T088 Configure Railway secrets: set environment variables from GitHub encrypted secrets

### Pre-Production Checklist

- [ ] T089 Verify all 17 functional requirements (FR-001 to FR-017) have corresponding implemented code
- [ ] T090 [P] Verify all 10 success criteria (SC-001 to SC-010) are measurable and testable
- [ ] T091 Manual testing: walk through all 3 user story flows end-to-end
- [ ] T092 [P] Security check: confirm API keys not in code, CORS properly restricted, rate limiting active
- [ ] T093 Load test: verify system handles 10+ concurrent /ask requests without degradation

### Deployment Steps

- [ ] T094 Create PR from 005-rag-chatbot-integration to main with testing checklist
- [ ] T095 Get code review: at least 1 approval from project lead
- [ ] T096 [P] Deploy backend to Railway: push main branch triggers auto-deploy
- [ ] T097 [P] Deploy frontend: Vercel auto-deploys on main push
- [ ] T098 Post-deployment validation: smoke test all endpoints in production, verify health check passes

---

## Test Categories (Optional - Create if TDD Requested)

### Unit Tests
- Embedding generation (mock OpenAI)
- Chunking logic (verify token counts)
- Source extraction (verify format)
- Model validation (Pydantic)

### Integration Tests
- RAG pipeline end-to-end (mock LLM, real DB)
- API endpoint contract tests (mock RAG pipeline)
- Ingestion pipeline (markdown → Neon/Qdrant)

### E2E Tests
- User asks question → gets answer with sources
- Chat history persists and loads
- Selected text context is used

---

## Independent Test Criteria by User Story

### User Story 1 (Core Q&A)
✅ Independent test: Question submitted → Answer displayed with sources within 5 seconds

### User Story 2 (Selected Text)
✅ Independent test: Select text → Submit with context → Answer references selection

### User Story 3 (Chat History)
✅ Independent test: Multiple questions asked → History panel loads previous Q&A pairs

---

## Task Dependencies & Parallel Execution

### No Dependencies (Can start immediately)
- T001-T009: Setup tasks (all parallelizable)
- T010-T017: Database & config (all parallelizable)

### Blocking Dependencies
- T010, T011 must complete before T014, T015
- T018-T024 must complete before T026, T030

### Parallelizable Phases
- Phase 3 backend & Phase 4 frontend can develop in parallel (API contract defined)
- Phase 5 & 6 can develop in parallel after Phase 3
- Tests (T052-T055) can write during backend development

---

## Definition of Done (Per Task)

1. **Code written** and follows Python/JavaScript style guide (PEP 8, ESLint)
2. **Tests pass** (if applicable): pytest for backend, Docusaurus build for frontend
3. **Documentation added**: Docstrings + comments for complex logic
4. **Linked to spec**: Task references FR/SC from spec.md
5. **No TODOs**: Code complete, no placeholder comments

---

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|------------|
| Code Coverage | >85% | pytest coverage report |
| Latency p95 | <5s | Load test with 10 concurrent requests |
| Success Rate | >95% | Chat log success count / total |
| Build Time | <2m | GitHub Actions workflow duration |
| Documentation | 100% | All public functions have docstrings |

---

## Milestones

| Milestone | Tasks | Duration |
|-----------|-------|----------|
| MVP Complete | T001-T055 (Phase 1-4) | 2-3 weeks |
| US2 Complete | T056-T065 | +1 week |
| US3 Complete | T066-T075 | +1 week |
| Deployment Ready | T076-T098 | +3-5 days |

---

## Next Steps

1. Review this tasks.md with team
2. Estimate effort per task (story points)
3. Create GitHub issues from tasks (one issue per task)
4. Assign to developers
5. Start Phase 1 setup tasks (parallelizable)
6. After Phase 2: branch Phase 3 backend + Phase 4 frontend development in parallel
7. Run `/sp.phr` to record this task generation in history

**Ready to start implementation!** First step: Run Phase 1 setup tasks in parallel.
