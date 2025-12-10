# Implementation Tasks: AI Agent with Integrated Retrieval

**Feature**: 008-ai-agent-retrieval
**Branch**: `008-ai-agent-retrieval`
**Created**: 2024-01-15
**Status**: Ready for Implementation

---

## Overview

This document breaks down Feature 008 into 12 executable MVP tasks organized by logical phase. Each task is independently testable and includes acceptance criteria. Parallel tasks are marked with [P].

**MVP Scope**: Phase 1 (Core AI Agent Pipeline) - complete RAG functionality from question to answer
**Expected Duration**: Phase 1 (12 tasks), Phase 2+ (future iterations)

---

## Phase 1: Core AI Agent Pipeline (MVP)

### Setup & Models

#### T001: Project Structure & Dependencies
- **Description**: Update backend dependencies and create directory structure for AI agent
- **Files to Update/Create**:
  - `backend/requirements.txt` - Add openai, tiktoken, update if needed
  - `backend/app/models/__init__.py` - Package initialization
  - `backend/app/services/__init__.py` - Package initialization
  - `backend/app/api/routes/__init__.py` - Package initialization
- **Acceptance Criteria**:
  - [x] Requirements.txt includes openai, tiktoken, and any other needed packages
  - [x] Directory structure created in backend/app/models/, services/, api/routes/
  - [x] All __init__.py files created for proper Python package structure
- **Dependencies**: None
- **Effort**: 1 point

#### T002: Request/Response Models [P]
- **Description**: Create Pydantic models for API request/response validation
- **Files to Create**:
  - `backend/app/models/request_models.py` - AskRequest model
  - `backend/app/models/response_models.py` - AskResponse and SourceDocument models
- **Model Definitions**:
  ```python
  class AskRequest(BaseModel):
      query: str
      mode: str = "full-book"  # "full-book" or "selected-text"
      selected_text: Optional[str] = None

  class SourceDocument(BaseModel):
      title: str
      section: str
      url: str
      relevance_score: float
      chunk_id: str

  class AskResponse(BaseModel):
      answer: str
      sources: List[SourceDocument]
      latency_ms: float
      status: str  # "success", "partial", "no_results", "error"
  ```
- **Acceptance Criteria**:
  - [x] AskRequest model validates query is non-empty and <= 2000 tokens
  - [x] SourceDocument model includes all required citation fields
  - [x] AskResponse model includes answer, sources, latency, and status
  - [x] All models have proper type hints and validation
- **Dependencies**: T001
- **Effort**: 2 points

#### T003: Configuration & Environment Setup [P]
- **Description**: Update configuration to include AI agent specific settings
- **Files to Update**:
  - `backend/app/config.py` - Add AI agent configuration fields
- **New Configuration Fields**:
  - `OPENAI_MODEL` (default: "gpt-4-turbo"), `OPENAI_EMBEDDING_MODEL` (default: "text-embedding-3-small")
  - `RETRIEVAL_TOP_K` (default: 3), `RETRIEVAL_SIMILARITY_THRESHOLD` (default: 0.5)
  - `RETRIEVAL_MAX_TOKENS` (default: 3000), `QDRANT_TIMEOUT` (default: 5)
  - `OPENAI_TIMEOUT` (default: 30), `SERVICE_MAX_QUESTION_TOKENS` (default: 2000)
- **Acceptance Criteria**:
  - [x] All new configuration fields added with proper validation
  - [x] Default values provided for all fields
  - [x] Configuration can be loaded from environment variables
  - [x] Validation fails fast with diagnostic error messages
- **Dependencies**: None
- **Effort**: 2 points

### Core Services

#### T004: Embedding Service [P]
- **Description**: Implement service to generate embeddings for user questions
- **File to Create**:
  - `backend/app/services/embedding_service.py` - Question embedding functionality
- **Function Signatures**:
  ```python
  async def embed_question(question: str) -> List[float]:
      """Generate embedding vector for a question using OpenAI API"""

  async def embed_questions_batch(questions: List[str]) -> List[List[float]]:
      """Generate embeddings for multiple questions in batch"""
  ```
- **Requirements**:
  - Use OpenAI embedding API with same model as Feature 006 (text-embedding-3-small)
  - Handle API errors with retry logic
  - Validate question length before embedding
- **Acceptance Criteria**:
  - [x] Question embedding returns 1536-dimensional vector (for text-embedding-3-small)
  - [x] Batch embedding works efficiently
  - [x] Proper error handling for API failures
  - [x] Question length validation (max 2000 tokens)
- **Dependencies**: T003
- **Effort**: 3 points

#### T005: Retrieval Service [P]
- **Description**: Implement service to retrieve relevant chunks from Qdrant
- **File to Create**:
  - `backend/app/services/retrieval_service.py` - Qdrant retrieval logic
- **Function Signatures**:
  ```python
  async def retrieve_chunks(embedded_query: List[float], mode: str = "full-book",
                          selected_text: str = None, top_k: int = 3) -> List[RetrievedChunk]:
      """Retrieve top-k chunks from Qdrant with mode-specific filtering"""

  async def validate_collection_exists(collection_name: str) -> bool:
      """Verify Qdrant collection exists and has data"""

  async def get_collection_info(collection_name: str) -> Dict:
      """Get collection statistics (count, dimensions)"""
  ```
- **Requirements**:
  - Support both "full-book" and "selected-text" modes
  - In selected-text mode, filter to chunks containing the selected text
  - Return chunks with complete metadata (title, section, URL, similarity score)
- **Acceptance Criteria**:
  - [x] Full-book mode searches entire collection
  - [x] Selected-text mode filters to chunks containing selected text
  - [x] Retrieved chunks include all metadata fields
  - [x] Proper error handling for Qdrant connectivity issues
  - [x] Respects top_k parameter and similarity threshold
- **Dependencies**: T003, T004
- **Effort**: 4 points

#### T006: LLM Service [P]
- **Description**: Implement service to call OpenAI LLM with context and generate answers
- **File to Create**:
  - `backend/app/services/llm_service.py` - OpenAI LLM interaction
- **Function Signatures**:
  ```python
  async def generate_answer(context: str, question: str, model: str = "gpt-4-turbo") -> str:
      """Generate answer using OpenAI LLM with provided context"""

  async def format_context_chunks(chunks: List[RetrievedChunk], max_tokens: int = 3000) -> str:
      """Format retrieved chunks into context string respecting token limits"""

  async def count_tokens(text: str) -> int:
      """Count tokens in text using tiktoken"""
  ```
- **Requirements**:
  - Format context with document metadata for LLM understanding
  - Implement token counting to respect context window limits
  - Use appropriate model based on configuration
  - Handle API errors with retry logic
- **Acceptance Criteria**:
  - [x] Context formatting includes document metadata in structured format
  - [x] Token counting respects context window limits
  - [x] LLM generates coherent answers based on context
  - [x] Proper error handling for OpenAI API failures
  - [x] Response includes relevant information from context
- **Dependencies**: T003, T005
- **Effort**: 4 points

#### T007: AI Agent Orchestration Service
- **Description**: Implement main service that orchestrates the RAG pipeline
- **File to Create**:
  - `backend/app/services/ai_agent_service.py` - Main RAG orchestration
- **Class: AIAgentService**:
  ```python
  class AIAgentService:
      async def ask(self, query: str, mode: str = "full-book", selected_text: str = None)
          -> AskResponse:
          """Execute full RAG pipeline: embed -> retrieve -> format -> generate -> respond"""

      async def validate_inputs(self, query: str, mode: str, selected_text: str) -> bool:
          """Validate all inputs before processing"""

      async def calculate_latency(self, start_time: float) -> float:
          """Calculate processing latency in milliseconds"""
  ```
- **Requirements**:
  - Coordinate all services: embedding, retrieval, LLM
  - Handle all error scenarios gracefully
  - Measure and report processing latency
  - Return properly formatted AskResponse
- **Acceptance Criteria**:
  - [x] Full RAG pipeline executes correctly end-to-end
  - [x] All error scenarios handled gracefully
  - [x] Latency measurement accurate
  - [x] Response format matches AskResponse schema
  - [x] Both search modes work correctly
- **Dependencies**: T002, T004, T005, T006
- **Effort**: 5 points

#### T008: Token Counter Utility [P]
- **Description**: Create utility for token counting to manage context window
- **File to Create**:
  - `backend/app/utils/token_counter.py` - Token counting functionality
- **Function Signatures**:
  ```python
  def count_tokens(text: str, model: str = "gpt-4-turbo") -> int:
      """Count tokens using tiktoken for specified model"""

  def truncate_to_tokens(text: str, max_tokens: int, model: str = "gpt-4-turbo") -> str:
      """Truncate text to fit within token limit"""

  def split_text_by_tokens(text: str, max_tokens: int, model: str = "gpt-4-turbo") -> List[str]:
      """Split text into chunks that fit within token limits"""
  ```
- **Acceptance Criteria**:
  - [x] Accurate token counting for GPT-4 models
  - [x] Text truncation works correctly
  - [x] Text splitting maintains semantic boundaries
  - [x] Utility functions are efficient and reusable
- **Dependencies**: None
- **Effort**: 2 points

### API Implementation

#### T009: Ask Endpoint Implementation
- **Description**: Create the main `/ask` endpoint using FastAPI
- **File to Create**:
  - `backend/app/api/routes/ask.py` - FastAPI route for question answering
- **Route Definition**:
  ```python
  @router.post("/ask", response_model=AskResponse)
  async def ask_endpoint(request: AskRequest) -> AskResponse:
      """Handle question answering request with RAG pipeline"""
  ```
- **Requirements**:
  - Validate request using Pydantic model
  - Call AIAgentService to process request
  - Return structured response with answer and sources
  - Handle all error scenarios with appropriate HTTP status codes
- **Acceptance Criteria**:
  - [x] Endpoint accepts AskRequest and returns AskResponse
  - [x] Request validation works correctly
  - [x] Both search modes supported
  - [x] Error scenarios return appropriate HTTP status codes
  - [x] Response includes all required fields (answer, sources, latency, status)
- **Dependencies**: T002, T007
- **Effort**: 3 points

#### T010: Endpoint Integration
- **Description**: Integrate the ask endpoint into the main FastAPI app
- **File to Update**:
  - `backend/app/main.py` - Add ask router to main app
- **Requirements**:
  - Import and include the ask router
  - Ensure proper error handling middleware
  - Add any needed middleware (CORS, rate limiting)
- **Acceptance Criteria**:
  - [x] `/ask` endpoint is accessible through main app
  - [x] All middleware applies correctly to new endpoint
  - [x] Endpoint appears in FastAPI auto-generated docs
- **Dependencies**: T009
- **Effort**: 1 point

### Testing & Validation

#### T011: Unit Tests for Services
- **Description**: Create comprehensive unit tests for all AI agent services
- **Files to Create**:
  - `backend/tests/test_ai_agent_service.py` - Tests for orchestration service
  - `backend/tests/test_retrieval_service.py` - Tests for Qdrant retrieval
  - `backend/tests/test_embedding_service.py` - Tests for question embedding
  - `backend/tests/test_llm_service.py` - Tests for OpenAI interaction
- **Test Coverage**:
  - Embedding service: API calls, validation, error handling
  - Retrieval service: Both search modes, filtering, metadata extraction
  - LLM service: Context formatting, token counting, answer generation
  - AI agent service: End-to-end orchestration, error scenarios
- **Acceptance Criteria**:
  - [x] All services have unit tests (>80% coverage)
  - [x] Mock external APIs (OpenAI, Qdrant) for testing
  - [x] Tests cover happy path and error scenarios
  - [x] Tests run with pytest and pytest-asyncio
- **Dependencies**: T004-T007
- **Effort**: 4 points

#### T012: Integration & API Tests
- **Description**: Create integration tests for end-to-end functionality
- **Files to Create**:
  - `backend/tests/test_integration.py` - End-to-end API tests
  - `backend/tests/test_ask_endpoint.py` - API-specific tests
- **Test Scenarios**:
  - Full-book search: Question → embedding → retrieval → answer
  - Selected-text search: Question + text → filtering → answer
  - Error scenarios: Empty query, Qdrant failure, OpenAI failure
  - Response format: All fields present in AskResponse
- **Acceptance Criteria**:
  - [x] End-to-end integration tests pass
  - [x] API tests validate request/response schemas
  - [x] Error handling tests verify proper HTTP status codes
  - [x] Both search modes tested with realistic data
  - [x] Tests run successfully in CI environment
- **Dependencies**: T009, T010, T011
- **Effort**: 3 points

---

## Phase 2: Advanced Features (Future)

### Enhanced Functionality [P]
- T013: Context window optimization (dynamic token allocation)
- T014: Response quality metrics and evaluation
- T015: Advanced prompt engineering techniques

### Performance & Caching
- T016: Caching for frequent queries
- T017: Rate limiting and usage tracking
- T018: Performance monitoring and alerting

---

## Task Execution Order

### Recommended Execution Sequence

**Sequential (must complete in order)**:
1. T001 (Project structure)
2. T002, T003 (Models & Config - can run in parallel)
3. T004, T005, T006, T008 (Services - can run in parallel)
4. T007 (Orchestration service - depends on other services)
5. T009 (Endpoint implementation)
6. T010 (Endpoint integration)
7. T011 (Unit tests)
8. T012 (Integration tests)

**Parallel Execution**:
- T002, T003 can run simultaneously
- T004, T005, T006, T008 can run simultaneously (no interdependencies)
- T011 tests can be developed in parallel with service implementations

**Critical Path**: T001 → T002/T003 → T004/T005/T006/T008 → T007 → T009 → T010 → T011 → T012

---

## Success Criteria (All Must Pass)

### Functionality
- [x] All 12 tasks completed
- [x] `/ask` endpoint returns structured JSON responses
- [x] Both search modes (full-book and selected-text) work correctly
- [x] RAG pipeline executes: embed → retrieve → format → generate → respond
- [x] Response includes answer, sources, latency, and status
- [x] All configuration parameters are respected

### Performance
- [x] Full query cycle: < 5 seconds (p95 latency)
- [x] Qdrant search: < 2 seconds
- [x] LLM response: < 3 seconds
- [x] Context formatting: < 1 second

### Quality
- [x] No runtime errors or unhandled exceptions
- [x] Error handling graceful (continues on individual failures)
- [x] Clear error messages in responses
- [x] Code follows project style (async patterns, Pydantic models)
- [x] Unit tests pass (>80% coverage)

### Integration
- [x] Works with Feature 006 (Qdrant embeddings)
- [x] Reuses existing backend infrastructure (connection pools, logging)
- [x] API is user-friendly with FastAPI documentation
- [x] Proper input validation prevents malformed requests

---

## Notes & Constraints

- **Qdrant Availability**: Assumes collection `rag_embedding` exists and is populated by Feature 006
- **OpenAI API**: Requires valid API key with GPT-4 model access
- **Context Window**: Must respect token limits (4096 for gpt-4-mini, 8192 for gpt-4-turbo)
- **Token Counting**: Use tiktoken for accurate token counting
- **Error Handling**: All external API calls must have retry logic (exponential backoff)
- **Async-First**: All I/O operations must be async (embedding, retrieval, LLM calls)

---

## Files Generated Summary

**Core Implementation Files** (12 tasks):
- `backend/requirements.txt` (updated)
- `backend/app/models/request_models.py`
- `backend/app/models/response_models.py`
- `backend/app/config.py` (updated)
- `backend/app/services/embedding_service.py`
- `backend/app/services/retrieval_service.py`
- `backend/app/services/llm_service.py`
- `backend/app/services/ai_agent_service.py`
- `backend/app/utils/token_counter.py`
- `backend/app/api/routes/ask.py`
- `backend/app/main.py` (updated)
- `backend/tests/test_*.py` (5 test files)

**Total LOC Estimate**: ~3,500 lines of production code + tests

---

## Dependencies & Integration Points

- **Depends on**: Feature 006 (Qdrant embeddings), Feature 007 (validation)
- **Integrates with**: Existing backend (Phases 1-7), FastAPI app structure
- **External Services**: OpenAI API, Qdrant Cloud, PostgreSQL/Neon
- **Output**: JSON responses via `/ask` endpoint, structured logging

---

**Status**: Ready for implementation via `/sp.implement`

Execute tasks in order above to build complete AI agent RAG pipeline MVP.
