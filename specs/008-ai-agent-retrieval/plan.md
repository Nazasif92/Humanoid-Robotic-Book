# Implementation Plan: AI Agent with Integrated Retrieval

**Branch**: `008-ai-agent-retrieval` | **Date**: 2024-01-15 | **Spec**: [specs/008-ai-agent-retrieval/spec.md](spec.md)
**Input**: Feature specification + planning requirements for FastAPI AI agent with Qdrant retrieval

## Summary

Create a RAG (Retrieval-Augmented Generation) AI Agent that enables users to ask natural language questions about the Humanoid Robotic Book. The system will expose a FastAPI `/ask` endpoint that accepts user queries, performs similarity search in Qdrant to retrieve relevant document chunks, formats these chunks into a context prompt for an OpenAI LLM, calls the model to generate an answer based on the retrieved context, and returns a structured JSON response with the answer, source citations, and performance metrics. The agent supports two modes: full-book search (default) and selected-text-only search for focused queries.

## Technical Context

**Language/Version**: Python 3.11+ (async/await patterns, type hints, Pydantic v2)

**Primary Dependencies**:
- `fastapi` - Web framework for API endpoints
- `uvicorn` - ASGI server for running FastAPI app
- `qdrant-client[http]` - Vector database client with async support
- `asyncpg` - Async PostgreSQL driver (reuse from existing backend)
- `openai` - OpenAI SDK for embeddings and LLM calls
- `pydantic` - Data validation and request/response models
- `python-dotenv` - Environment variable management
- `tiktoken` - Token counting for context window management

**Storage**:
- Qdrant Cloud - Vector storage with metadata payloads (rag_embedding collection)
- PostgreSQL (Neon) - Chunk metadata (documents, chunks tables from Feature 006)
- Local filesystem - Configuration and logging

**Testing**: pytest + pytest-asyncio for async test cases; mock OpenAI/Qdrant for unit tests

**Target Platform**: Linux server deployment (Railway); development on Windows/Linux/macOS

**Project Type**: Single Python web application (FastAPI) with async architecture

**Performance Goals**:
- Response latency: < 5 seconds (p95) for complete query cycle
- Concurrent requests: Support 10+ simultaneous users
- Qdrant search: < 2 seconds for top-k retrieval
- LLM response: < 3 seconds for answer generation

**Constraints**:
- OpenAI API rate limiting (handle gracefully with backoff)
- Context window limits (4096 tokens for gpt-4-mini, 8192 for gpt-4-turbo)
- Qdrant API rate limiting (handle gracefully with backoff)
- Memory usage: < 500MB for context formatting operations

**Scale/Scope**:
- ~1200+ documentation chunks from Feature 006
- 100+ concurrent users in production
- 2 search modes: full-book (default) and selected-text-only
- Top-k results: default k=3, configurable up to k=10

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on **Humanoid Robotic Book Constitution**:

| Principle | Check | Status | Notes |
|-----------|-------|--------|-------|
| **I. Accuracy & Correctness** | RAG logic mirrors actual search behavior; accuracy metrics are verifiable against real systems | ✅ PASS | Context injection, LLM calling, source attribution all testable |
| **II. Beginner-Friendly Language** | Code examples in README; error messages are diagnostic; API responses clear | ✅ PASS | JSON responses include sources; CLI help text clear; examples provided |
| **III. Consistent Style & Structure** | Follows existing backend patterns (async, Pydantic, error handling, logging) | ✅ PASS | Mirrors backend architecture (app/rag_pipeline.py, neon_client.py patterns) |
| **IV. Example-Driven Explanations** | API examples in README; includes sample requests/responses | ✅ PASS | FastAPI auto-generated docs; curl examples; response format documented |
| **V. Quality Review & Fact-Check** | All RAG logic testable; response schema validated; metrics defined | ✅ PASS | Tests for accuracy calculation, source attribution, error handling |

**Gate Status**: ✅ **PASS** - Feature aligns with all 5 constitution principles

## Project Structure

### Documentation (this feature)

```text
specs/008-ai-agent-retrieval/
├── spec.md                          # Feature specification (user stories, requirements, success criteria)
├── plan.md                          # This file (architecture and technical decisions)
├── checklists/
│   └── requirements.md              # Quality validation checklist
└── (future phases)
    ├── research.md                  # Phase 0: research on RAG approaches
    ├── data-model.md                # Phase 1: detailed entity definitions
    └── quickstart.md                # Phase 1: getting started guide
```

### Source Code (integrated with existing backend)

```text
backend/
├── app/
│   ├── main.py                      # FastAPI app entry point (existing)
│   ├── config.py                    # Configuration & environment variables (existing)
│   ├── models/                      # Pydantic models
│   │   ├── request_models.py        # AskRequest model
│   │   └── response_models.py       # AskResponse model
│   ├── services/                    # Core business logic
│   │   ├── ai_agent_service.py      # Main AI agent orchestration
│   │   ├── retrieval_service.py     # Qdrant retrieval logic
│   │   ├── embedding_service.py     # Question embedding generation
│   │   └── llm_service.py           # OpenAI LLM interaction
│   ├── api/
│   │   └── routes/
│   │       └── ask.py               # /ask endpoint definition
│   ├── utils/
│   │   ├── token_counter.py         # Context window management
│   │   └── logger.py                # Structured logging (existing)
│   └── tests/
│       ├── test_ai_agent_service.py # Unit tests for AI agent logic
│       ├── test_retrieval_service.py # Unit tests for retrieval
│       ├── test_embedding_service.py # Unit tests for embedding
│       ├── test_llm_service.py      # Unit tests for LLM interaction
│       └── test_integration.py      # End-to-end integration tests
└── requirements.txt                 # Updated dependencies

specs/008-ai-agent-retrieval/
├── spec.md
├── plan.md
└── checklists/requirements.md
```

**Structure Decision**: Integrate AI agent functionality into existing backend structure to reuse infrastructure (connection pools, configuration, logging, monitoring) rather than creating separate service. Follow existing patterns from Phases 1-7 for consistency. New modules added to existing backend structure.

## Complexity Tracking

*No Constitution Check violations detected. Design choices are justified by simplicity and consistency with existing architecture.*

---

## Architectural Decisions

### Decision 1: FastAPI Application Structure and Endpoint Design

**Choice**: Extend existing backend FastAPI app with new `/ask` route using Pydantic request/response models

**Rationale**:
- Reuses existing backend infrastructure (middleware, error handling, logging)
- Consistent with existing API patterns in the project
- Pydantic models provide automatic validation and documentation
- Single codebase reduces deployment complexity

**Implementation**:
```python
# backend/app/api/routes/ask.py
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

class AskRequest(BaseModel):
    query: str
    mode: str = "full-book"  # "full-book" or "selected-text"
    selected_text: str = None

class AskResponse(BaseModel):
    answer: str
    sources: List[SourceDocument]
    latency_ms: float
    status: str

@router.post("/ask", response_model=AskResponse)
async def ask_endpoint(request: AskRequest):
    # Implementation in AI Agent service
    pass
```

**Alternatives Considered**:
- Separate microservice: More complex deployment; harder to maintain
- GraphQL endpoint: Overkill for simple question-answering use case
- Direct function call: No web interface; limited to internal use

---

### Decision 2: Two-Mode Retrieval Strategy (Full-book vs Selected-text)

**Choice**: Implement mode-based retrieval with different filtering strategies for Qdrant search

**Rationale**:
- Full-book mode: Search entire Qdrant collection (default behavior)
- Selected-text mode: Filter results to chunks that contain selected text
- Single endpoint with mode parameter reduces API complexity
- Mode-specific logic handles different retrieval requirements

**Implementation**:
```python
async def retrieve_chunks(query: str, mode: str = "full-book", selected_text: str = None):
    if mode == "full-book":
        # Search entire collection
        results = await qdrant_client.search(
            collection_name="rag_embedding",
            query_vector=embedded_query,
            limit=top_k
        )
    elif mode == "selected-text":
        # Filter to chunks containing selected_text
        results = await qdrant_client.search(
            collection_name="rag_embedding",
            query_vector=embedded_query,
            limit=top_k,
            query_filter=Filter(
                must=[FieldCondition(
                    key="chunk_text",
                    match=MatchText(text=selected_text)
                )]
            )
        )
    return results
```

**Alternatives Considered**:
- Separate endpoints: `/ask-full` vs `/ask-selected` - increases API surface
- Different query parameters: More complex validation logic
- Client-side filtering: Less efficient; more data transfer

---

### Decision 3: Context Window Management and Prompt Engineering

**Choice**: Format retrieved chunks into structured context with document metadata, implement token counting to respect context limits

**Rationale**:
- Include document title, section, and URL in context for LLM to reference
- Token counting ensures context fits within model limits (4096/8192 tokens)
- Structured format helps LLM understand document boundaries
- Source attribution enables response citations

**Context Format**:
```
[DOCUMENT: "ROS 2 Basics" | SECTION: "Introduction" | URL: /docs/ros2-basics]
Content: ROS 2 (Robot Operating System 2) is a flexible framework for writing robot applications...

[DOCUMENT: "ROS 2 Architecture" | SECTION: "Nodes" | URL: /docs/ros2-architecture]
Content: Nodes are the fundamental building blocks of ROS 2 applications...
```

**Implementation**:
```python
def format_context_chunks(chunks: List[RetrievedChunk], max_tokens: int = 3000) -> str:
    """Format chunks into context string respecting token limits"""
    context_parts = []
    total_tokens = 0

    for chunk in chunks:
        chunk_text = f"[DOCUMENT: \"{chunk.title}\" | SECTION: \"{chunk.section}\" | URL: {chunk.url}]\nContent: {chunk.text}\n\n"
        chunk_tokens = count_tokens(chunk_text)

        if total_tokens + chunk_tokens > max_tokens:
            break

        context_parts.append(chunk_text)
        total_tokens += chunk_tokens

    return "".join(context_parts)
```

**Alternatives Considered**:
- Raw text concatenation: LLM might confuse document boundaries
- JSON format: Less readable for LLM; harder to parse
- Fixed-size chunks: Less efficient use of context window

---

### Decision 4: OpenAI Model Selection and Response Processing

**Choice**: Use GPT-4-Turbo for best quality, with GPT-4-Mini as fallback for cost/performance; implement structured response extraction

**Rationale**:
- GPT-4-Turbo: Higher quality responses, better at following instructions
- GPT-4-Mini: Lower cost, faster responses for simpler queries
- Structured extraction enables reliable source attribution
- Fallback mechanism handles rate limits gracefully

**Implementation**:
```python
async def generate_answer(context: str, question: str, model: str = "gpt-4-turbo"):
    prompt = f"""
    You are an expert assistant for the Humanoid Robotic Book.
    Answer the user's question based on the provided context.

    Context:
    {context}

    Question: {question}

    Instructions:
    - Answer directly based on the context
    - If the context doesn't contain relevant information, say so
    - Cite specific documents when possible
    - Keep answers concise but comprehensive
    """

    response = await openai_client.chat.completions.create(
        model=model,
        messages=[{"role": "user", "content": prompt}],
        temperature=0.3,
        max_tokens=1000
    )

    return response.choices[0].message.content
```

**Alternatives Considered**:
- OpenAI Functions: More structured but adds complexity
- Custom prompt templates: Less flexible than standard approach
- Different models: GPT-3.5 too limited; GPT-4 more expensive without benefit

---

### Decision 5: Error Handling and Resilience Strategy

**Choice**: Implement comprehensive error handling with retry logic, graceful degradation, and diagnostic logging

**Rationale**:
- OpenAI API can be unreliable; retry logic improves availability
- Qdrant connectivity issues should not crash the service
- Diagnostic logging enables quick debugging
- Graceful degradation maintains basic functionality when components fail

**Error Handling Strategy**:
- **OpenAI failures**: Exponential backoff (1s, 2s, 4s) with max 3 attempts
- **Qdrant failures**: Return informative message if search fails
- **Validation failures**: Return 400 Bad Request with specific error
- **Timeouts**: 5s for Qdrant, 30s for OpenAI to prevent hanging requests

**Implementation**:
```python
async def robust_ask(query: str, mode: str = "full-book"):
    try:
        # Embed question with retry
        embedded_query = await retry_with_backoff(
            lambda: embed_question(query),
            max_attempts=3
        )

        # Retrieve chunks with timeout
        chunks = await asyncio.wait_for(
            retrieve_chunks(embedded_query, mode),
            timeout=5.0
        )

        if not chunks:
            return AskResponse(
                answer="No relevant information found for your query.",
                sources=[],
                latency_ms=0,
                status="no_results"
            )

        # Generate answer with retry
        answer = await retry_with_backoff(
            lambda: generate_answer(format_context(chunks), query),
            max_attempts=2
        )

        return AskResponse(
            answer=answer,
            sources=[SourceDocument.from_chunk(c) for c in chunks],
            latency_ms=calculate_latency(),
            status="success"
        )

    except asyncio.TimeoutError:
        raise HTTPException(status_code=504, detail="Service timeout")
    except Exception as e:
        logger.error(f"Error in ask endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")
```

**Alternatives Considered**:
- Simple try/catch: Less granular error handling
- No retries: Lower availability when APIs are temporarily unavailable
- Generic error responses: Less helpful for debugging

---

## Data Model

### Entities

**1. AskRequest**
- `query`: User's question string (required, non-empty, max 2000 tokens)
- `mode`: Search mode ("full-book" or "selected-text", default "full-book")
- `selected_text`: Optional text for context-limited search (when mode="selected-text")

**2. AskResponse**
- `answer`: Generated answer string from LLM
- `sources`: List of SourceDocument objects citing used documents
- `latency_ms`: Total processing time in milliseconds
- `status`: Operation status ("success", "partial", "no_results", "error")

**3. SourceDocument**
- `title`: Document title from metadata
- `section`: Section name within document
- `url`: Docusaurus URL for the document
- `relevance_score`: Similarity score from Qdrant search
- `chunk_id`: Unique identifier for the retrieved chunk

**4. RetrievedChunk**
- `chunk_id`: Unique identifier from Qdrant
- `similarity_score`: Cosine similarity (0.0-1.0)
- `text`: Full text of the chunk
- `title`: Document title
- `section`: Section name
- `url`: Docusaurus URL
- `document_id`: Parent document identifier

## API Contracts

### FastAPI Endpoint (`/ask`)

```python
POST /ask

Request Body (JSON):
{
  "query": "What is ROS 2?",
  "mode": "full-book",           # Optional: "full-book" (default) or "selected-text"
  "selected_text": "Optional text for context-limited search"  # Required when mode="selected-text"
}

Response (JSON):
{
  "answer": "ROS 2 (Robot Operating System 2) is a flexible framework...",
  "sources": [
    {
      "title": "ROS 2 Basics",
      "section": "Introduction",
      "url": "/docs/ros2-basics",
      "relevance_score": 0.87,
      "chunk_id": "doc_42_chunk_0"
    }
  ],
  "latency_ms": 2450,
  "status": "success"
}

Error Responses:
- 400: Bad Request (invalid query, missing required fields)
- 408: Request Timeout (service timeout)
- 429: Too Many Requests (rate limiting)
- 500: Internal Server Error (system failure)
- 502: Bad Gateway (upstream service failure)
- 504: Gateway Timeout (upstream timeout)
```

### Environment Variables

```bash
# OpenAI Configuration
OPENAI_API_KEY=your-api-key
OPENAI_MODEL=gpt-4-turbo          # Default model
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key
QDRANT_COLLECTION_NAME=rag_embedding

# Retrieval Configuration
RETRIEVAL_TOP_K=3                 # Number of chunks to retrieve
RETRIEVAL_SIMILARITY_THRESHOLD=0.5 # Minimum similarity score
RETRIEVAL_MAX_TOKENS=3000         # Maximum context window tokens

# Timeout Configuration
QDRANT_TIMEOUT=5                  # Qdrant search timeout (seconds)
OPENAI_TIMEOUT=30                 # OpenAI API timeout (seconds)

# Service Configuration
SERVICE_MAX_QUESTION_TOKENS=2000  # Maximum question length
SERVICE_RETRY_ATTEMPTS=3          # API retry attempts
```

### Output Contract

All successful responses follow the AskResponse schema with consistent field presence:
- `answer`: Always present (may be "No relevant information found" if no results)
- `sources`: Array of SourceDocument objects (empty array if no sources used)
- `latency_ms`: Processing time in milliseconds
- `status`: Operation status indicator

---

## Implementation Phases

### Phase 1: Core AI Agent Pipeline (MVP)

**Tasks**:
1. Request/response models (AskRequest, AskResponse, SourceDocument)
2. Configuration module with environment validation
3. Qdrant client integration and connection management
4. Question embedding service with OpenAI
5. Retrieval service with two-mode filtering
6. LLM service with GPT-4 integration
7. AI agent orchestration service
8. `/ask` endpoint implementation
9. Basic error handling and logging
10. Unit tests for each component
11. Integration test for end-to-end flow
12. README with API examples

**Deliverables**:
- backend/app/models/request_models.py, response_models.py
- backend/app/services/ai_agent_service.py, retrieval_service.py, embedding_service.py, llm_service.py
- backend/app/api/routes/ask.py
- backend/app/utils/token_counter.py
- Tests for all components
- API documentation

**Estimated**: 12 tasks, ~60 story points

### Phase 2: Advanced Features (Future)

- Context window optimization (dynamic token allocation)
- Response quality metrics and evaluation
- Advanced prompt engineering techniques
- Caching for frequent queries
- Rate limiting and usage tracking
- Performance monitoring and alerting

### Phase 3: Production Deployment (Future)

- Horizontal scaling configuration
- Advanced monitoring and observability
- A/B testing framework for prompt variations
- Advanced analytics and usage insights

---

## Success Criteria for Implementation

1. **Functionality**: `/ask` endpoint returns structured JSON with answer and sources
2. **Quality**: ≥80% of sample questions receive accurate, on-topic answers
3. **Performance**: <5s response time for 95% of requests (p95 latency)
4. **Reliability**: Handles OpenAI/Qdrant failures gracefully with retry logic
5. **Accuracy**: 100% source attribution accuracy (all cited sources actually used)
6. **Modes**: Both full-book and selected-text modes work correctly
7. **Validation**: Input validation prevents malformed requests
8. **Testing**: Unit tests cover 80%+ of code; integration tests verify end-to-end flow
9. **Documentation**: API examples and usage guide available
10. **Production Readiness**: Zero unhandled exceptions; comprehensive error logging

---

## Risks and Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|-----------|
| OpenAI API rate limiting during high usage | Medium | Service degradation | Implement request queuing; cache frequent queries; use GPT-4-mini for simple queries |
| Context window overflow with large retrieved chunks | Low | Answer quality degradation | Token counting with truncation; dynamic chunk selection |
| Qdrant collection not populated from Feature 006 | Medium | Feature completely non-functional | Validate collection exists at startup; provide clear error messages |
| Embedding model mismatch (different dimensions) | Low | Retrieval quality poor | Validate embedding dimensions match during initialization |
| Concurrent requests exceeding API limits | Medium | Service timeouts | Implement connection pooling; rate limiting at API gateway |

---

## Dependencies and Assumptions

**Dependencies**:
- Feature 006 (embedding pipeline) must be complete with Qdrant collection populated
- Feature 007 (retrieval validation) to ensure quality before deployment
- Existing backend infrastructure (Phases 1-7) for connection pools, logging, monitoring

**Assumptions**:
- OpenAI API access with sufficient rate limits for expected usage
- Qdrant Cloud cluster is properly configured and accessible
- Document embeddings from Feature 006 are of sufficient quality
- Context window of GPT-4 models is sufficient for retrieved chunks
- Users will primarily ask questions about the documented content
