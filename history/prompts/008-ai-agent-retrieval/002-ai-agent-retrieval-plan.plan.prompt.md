---
id: 002
title: AI Agent with Integrated Retrieval Architecture & Design
stage: plan
date: 2024-01-15
surface: agent
model: claude-haiku-4-5-20251001
feature: 008-ai-agent-retrieval
branch: 008-ai-agent-retrieval
user: user
command: /sp.plan
labels: ["architecture", "design", "ai-agent", "rag", "openai", "qdrant", "retrieval"]
links:
  spec: specs/008-ai-agent-retrieval/spec.md
  plan: specs/008-ai-agent-retrieval/plan.md
  adr: null
  pr: null
files:
  - specs/008-ai-agent-retrieval/plan.md
  - specs/008-ai-agent-retrieval/tasks.md
tests:
  - Constitution Check (PASSED)
  - Architecture Review (PASSED)
  - Task Breakdown (PASSED)
---

## Prompt

Create architecture design for AI agent with integrated retrieval:
- Initialize FastAPI app structure
- Create /ask endpoint with request model: {query, mode}
- Connect to Qdrant collection
- Run similarity search (top-k)
- Format retrieved chunks into RAG prompt
- Call OpenAI model with constructed context
- Return JSON: answer + metadata + sources
- Add logging + minimal tests

## Response snapshot

**Feature 008 Implementation Plan Complete**: Comprehensive architecture and technical design for AI Agent with Integrated Retrieval.

**Key Architectural Decisions**:
1. FastAPI Application Structure (extends existing backend with new /ask route using Pydantic models)
2. Two-Mode Retrieval Strategy (full-book vs selected-text with different Qdrant filtering)
3. Context Window Management (structured format with document metadata, token counting)
4. OpenAI Model Selection (GPT-4-Turbo for quality, GPT-4-Mini as fallback)
5. Error Handling Strategy (comprehensive retry logic, graceful degradation, diagnostic logging)

**Technical Stack**:
- Python 3.11 with async/await patterns
- FastAPI web framework with Pydantic validation
- qdrant-client[http] async driver for vector search
- asyncpg for PostgreSQL metadata (reuse existing pool)
- openai SDK for embeddings and LLM calls
- tiktoken for context window management
- pytest-asyncio for async testing

**Architecture**:
```
backend/
├── app/
│   ├── models/
│   │   ├── request_models.py      # AskRequest model
│   │   └── response_models.py     # AskResponse model
│   ├── services/
│   │   ├── ai_agent_service.py    # Main orchestration
│   │   ├── retrieval_service.py   # Qdrant retrieval
│   │   ├── embedding_service.py   # Question embedding
│   │   └── llm_service.py         # OpenAI interaction
│   ├── api/
│   │   └── routes/
│   │       └── ask.py             # /ask endpoint
│   ├── utils/
│   │   └── token_counter.py       # Context window management
│   └── tests/
│       └── (test files)
```

**API Contract**:
- POST /ask with JSON: {query, mode, selected_text}
- Response: {answer, sources[], latency_ms, status}
- Error codes: 400, 408, 429, 500, 502, 504
- Environment variables for all configuration

**Context Format**:
```
[DOCUMENT: "ROS 2 Basics" | SECTION: "Introduction" | URL: /docs/ros2-basics]
Content: ROS 2 (Robot Operating System 2) is a flexible framework...
```

**Output Contract**:
- JSON response with answer, sources (title, section, URL, relevance_score), latency, status
- Sources always match documents that contributed to answer
- Latency measured in milliseconds
- Status indicates success, partial, no_results, or error

**Implementation Phases**:
- Phase 1 (MVP): 12 tasks covering models, services, API, testing
- Phase 2: Advanced features (context optimization, caching, monitoring)
- Phase 3: Production deployment (scaling, observability)

**Success Criteria**:
1. /ask endpoint returns structured JSON with answer + sources
2. ≥80% question answering accuracy
3. <5s response time (p95 latency)
4. Both search modes work correctly
5. Comprehensive error handling
6. Unit + integration tests passing
7. Production-ready with zero unhandled exceptions

**Constitution Check**: ✅ PASSED - Aligns with all 5 project principles (Accuracy, Beginner-Friendly, Consistency, Example-Driven, Quality Review)

## Outcome

- ✅ Impact: Plan enables clear task generation and implementation. Establishes RAG AI agent that answers questions about Humanoid Robotic Book with source citations.
- 🧪 Tests: Constitution Check - PASS; Architecture Review - PASS; Task Breakdown - PASS
- 📁 Files: Generated plan.md (500+ lines) with 5 architectural decisions, detailed data model, API contracts; Generated tasks.md (400+ lines) with 12 executable MVP tasks with acceptance criteria
- 🔁 Next prompts: `/sp.tasks` (already executed) to generate detailed task breakdown; `/sp.implement` to begin implementation
- 🧠 Reflection: Plan balances simplicity (extends existing backend) with functionality (full RAG pipeline). Reuses Feature 006 infrastructure (Qdrant embeddings, PostgreSQL metadata). Clear two-mode search (full-book vs selected-text) enables both broad and focused queries. Context window management ensures LLM quality. Error handling strategy provides production reliability.

## Evaluation notes (flywheel)

- Failure modes observed: None - specification was clear, planning requirements well-scoped
- Graders run and results (PASS/FAIL): Constitution Check - PASS; Architecture Review - PASS; Task Breakdown - PASS
- Prompt variant (if applicable): User provided explicit implementation approach (FastAPI endpoint, Qdrant retrieval, OpenAI LLM, JSON response, two modes) which were directly incorporated into architecture decisions and task breakdown
- Next experiment (smallest change to try): Proceed directly to `/sp.implement` for Phase 1 implementation. Plan is comprehensive and ready for execution.
