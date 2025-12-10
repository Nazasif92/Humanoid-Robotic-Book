# Implementation Tasks: Retrieval Pipeline Validation

**Feature**: 007-retrieval-validation
**Branch**: `007-retrieval-validation`
**Created**: 2024-01-15
**Status**: Ready for Implementation

---

## Overview

This document breaks down Feature 007 into 10 executable MVP tasks organized by logical phase. Each task is independently testable and includes acceptance criteria. Parallel tasks are marked with [P].

**MVP Scope**: Phase 1 (Core Validation Pipeline) - complete validation tool from Qdrant connection to JSON report
**Expected Duration**: Phase 1 (10 tasks), Phase 2+ (future iterations)

---

## Phase 1: Core Validation Pipeline (MVP)

### Setup & Configuration

#### T001: Project Structure & Dependencies
- **Description**: Create test_retrieval/ directory structure with Python package setup
- **Files to Create**:
  - `test_retrieval/__init__.py` - Package initialization
  - `test_retrieval/requirements.txt` - Pinned dependencies (qdrant-client, asyncpg, numpy, pydantic, openai)
  - `test_retrieval/README.md` - Usage documentation with CLI examples
- **Acceptance Criteria**:
  - [x] Directory structure created at test_retrieval/
  - [x] requirements.txt includes all dependencies with versions
  - [x] README includes installation, CLI examples, and troubleshooting
- **Dependencies**: None
- **Effort**: 2 points

#### T002: Configuration & Environment Setup [P]
- **Description**: Create configuration module for environment variables and validation
- **Files to Create**:
  - `test_retrieval/config.py` - Settings class with validation
  - Update `.env.example` with retrieval-specific variables
- **Configuration Variables**:
  - `QDRANT_URL`, `QDRANT_API_KEY`, `QDRANT_COLLECTION_NAME`
  - `NEON_CONNECTION_STRING`
  - `OPENAI_API_KEY`, `OPENAI_EMBEDDING_MODEL`
  - `RETRIEVAL_SIMILARITY_THRESHOLD=0.75`, `RETRIEVAL_TOP_K=5`
- **Acceptance Criteria**:
  - [x] All environment variables validated with pydantic
  - [x] Default values provided (similarity_threshold=0.75, top_k=5)
  - [x] Configuration can be loaded from .env file
  - [x] Validation fails fast with diagnostic error messages
- **Dependencies**: None
- **Effort**: 2 points

#### T003: Qdrant Client Setup [P]
- **Description**: Implement Qdrant async client initialization and connection management
- **File to Create**:
  - `test_retrieval/qdrant_client_wrapper.py` - Wrapper for async Qdrant operations
- **Function Signatures**:
  ```python
  async def connect(url: str, api_key: str) -> AsyncClient:
      """Initialize Qdrant async client with timeout"""

  async def search_similar(client: AsyncClient, collection: str, vector: List[float],
                          top_k: int = 5, with_payload: bool = True) -> List[Dict]:
      """Search for similar vectors with full metadata payloads"""

  async def get_collection_info(client: AsyncClient, collection: str) -> Dict:
      """Retrieve collection statistics (count, vector dimension)"""

  async def health_check(client: AsyncClient) -> bool:
      """Verify Qdrant connection is healthy"""
  ```
- **Acceptance Criteria**:
  - [x] Async client connects successfully to Qdrant Cloud
  - [x] Search returns top-k results with metadata payloads
  - [x] Collection info includes point count and dimension
  - [x] Health check detects connection failures
  - [x] Timeout handling (30s default, configurable)
- **Dependencies**: None
- **Effort**: 3 points

#### T004: PostgreSQL Metadata Retrieval [P]
- **Description**: Implement async PostgreSQL queries to retrieve chunk metadata
- **File to Create**:
  - `test_retrieval/postgres_metadata.py` - PostgreSQL metadata queries
- **Function Signatures**:
  ```python
  async def get_chunk_metadata(pool: asyncpg.Pool, chunk_id: str) -> Dict:
      """Retrieve metadata for single chunk from PostgreSQL"""

  async def get_chunks_for_document(pool: asyncpg.Pool, doc_id: str) -> List[Dict]:
      """Retrieve all chunks for a document (validate ordering)"""

  async def get_all_chunks_count(pool: asyncpg.Pool) -> int:
      """Count total chunks in database"""

  async def validate_chunk_exists(pool: asyncpg.Pool, chunk_id: str) -> bool:
      """Check if chunk exists in PostgreSQL"""
  ```
- **Requirements**:
  - Reuse asyncpg from backend (share connection pool)
  - Query documents and chunks tables
  - Extract: title, section, url, path, chunk_index, token_count
- **Acceptance Criteria**:
  - [x] Queries return complete metadata records
  - [x] Chunk ordering by chunk_index is correct
  - [x] Handles missing chunks gracefully
  - [x] Uses existing connection pool from backend
- **Dependencies**: None (asyncpg already available)
- **Effort**: 2 points

### Core Validation Logic

#### T005: Test Query Management [P]
- **Description**: Create test query dataset and loading mechanism
- **Files to Create**:
  - `test_retrieval/test_queries.json` - 20+ test queries with expected keywords
  - `test_retrieval/query_loader.py` - Load and manage test queries
- **test_queries.json Structure**:
  ```json
  {
    "test_queries": [
      {
        "query_id": "q001",
        "text": "What is ROS 2?",
        "expected_keywords": ["ROS", "middleware", "robotics"],
        "relevance_threshold": 0.75,
        "expected_top_k": 3
      },
      {
        "query_id": "q002",
        "text": "How to install dependencies?",
        "expected_keywords": ["install", "setup", "dependencies"],
        "relevance_threshold": 0.70,
        "expected_top_k": 3
      }
    ]
  }
  ```
- **Function Signatures**:
  ```python
  def load_test_queries(file_path: str) -> List[TestQuery]:
      """Load test queries from JSON file"""

  def get_test_queries_by_ids(query_ids: List[str]) -> List[TestQuery]:
      """Filter test queries by IDs"""

  async def embed_test_queries(queries: List[TestQuery], openai_client) -> List[Tuple[TestQuery, List[float]]]:
      """Batch embed test queries using OpenAI"""
  ```
- **Acceptance Criteria**:
  - [x] test_queries.json contains 20+ diverse test queries
  - [x] Query IDs are unique (q001, q002, etc.)
  - [x] Keyword lists cover major documentation topics
  - [x] Can load, filter, and batch embed queries
  - [x] Handles missing/malformed query files gracefully
- **Dependencies**: None
- **Effort**: 3 points

#### T006: Retrieval & Metadata Validation [P]
- **Description**: Core validation logic: search, retrieve metadata, compare
- **File to Create**:
  - `test_retrieval/retrieval_validator.py` - Main validation logic class
- **Class: RetrievalValidator**:
  ```python
  class RetrievalValidator:
      async def validate_search_quality(self, query: TestQuery, results: List[SearchResult])
          -> SearchQualityMetrics:
          """Evaluate if results match expected keywords and similarity threshold"""

      async def validate_metadata_integrity(self, chunk_id: str, qdrant_metadata: Dict,
                                           postgres_metadata: Dict) -> ValidationResult:
          """Compare 5 critical fields: url, path, title, section, chunk_index"""

      async def validate_chunk_ordering(self, doc_id: str) -> ChunkOrderingResult:
          """Verify chunk_index sequence is correct within document"""

      async def check_missing_chunks(self, qdrant_chunks: Set[str],
                                    postgres_chunks: Set[str]) -> List[MissingChunk]:
          """Identify orphaned or missing chunks"""

      async def run_full_validation(self) -> ValidationReport:
          """Execute all validation checks end-to-end"""
  ```
- **Validation Fields** (metadata comparison):
  1. `url` - Constructed Docusaurus URL
  2. `path` - Source file path
  3. `title` - Document title
  4. `section` - Section name
  5. `chunk_index` - Sequence within document
- **Acceptance Criteria**:
  - [x] Search quality validated by similarity threshold and keyword matching
  - [x] All 5 metadata fields compared Qdrant vs PostgreSQL
  - [x] Chunk ordering verified for each document
  - [x] Missing chunks detected (Qdrant vs PostgreSQL mismatch)
  - [x] All validation results trackable for reporting
- **Dependencies**: T003, T004, T005
- **Effort**: 5 points

#### T007: Similarity Evaluation & Relevance Classification
- **Description**: Classify search results by cosine similarity threshold
- **File to Create**:
  - `test_retrieval/similarity_evaluator.py` - Similarity-based classification
- **Function Signatures**:
  ```python
  def classify_by_similarity(similarity_score: float, threshold: float = 0.75) -> str:
      """Return 'HIGHLY_RELEVANT' (≥0.80), 'RELEVANT' (0.70-0.79), or 'LOW_RELEVANCE'"""

  def calculate_relevance_metrics(results: List[SearchResult], threshold: float)
      -> RelevanceMetrics:
      """Count and classify results by relevance level"""

  def calculate_accuracy(results: List[SearchResult], expected_keywords: List[str],
                        threshold: float) -> float:
      """Percent of top-k results matching threshold and containing expected keywords"""
  ```
- **Relevance Thresholds**:
  - ≥0.80 = HIGHLY_RELEVANT (gold standard)
  - 0.70-0.79 = RELEVANT (acceptable)
  - <0.70 = LOW_RELEVANCE (failure)
- **Acceptance Criteria**:
  - [x] Classification correctly maps similarity scores to relevance levels
  - [x] Accuracy calculation includes both similarity and keyword matching
  - [x] Threshold is configurable (default 0.75)
  - [x] Handles edge cases (0 results, all low relevance)
- **Dependencies**: None
- **Effort**: 2 points

### Output & Reporting

#### T008: JSON Report Generation
- **Description**: Generate structured validation report in JSON format
- **File to Create**:
  - `test_retrieval/report_generator.py` - Report schema and generation
- **Report Schema** (retrieval_report.json):
  ```json
  {
    "status": "PASS|FAIL|PARTIAL",
    "timestamp": "ISO8601",
    "summary": {
      "total_queries_tested": int,
      "queries_passed": int,
      "queries_failed": int,
      "overall_accuracy_percent": float
    },
    "metrics": {
      "metadata_accuracy_percent": float,
      "url_accuracy_percent": float,
      "chunk_ordering_accuracy_percent": float,
      "avg_search_latency_ms": float,
      "total_chunks_validated": int
    },
    "query_results": [
      {
        "query_id": string,
        "status": "PASS|FAIL",
        "accuracy_percent": float,
        "top_k_results": [SearchResult]
      }
    ],
    "errors": [
      {
        "type": "MISSING_CHUNK|METADATA_MISMATCH|LOW_RELEVANCE",
        "chunk_id": string,
        "description": string
      }
    ]
  }
  ```
- **Function Signatures**:
  ```python
  async def generate_report(validation_results: ValidationResults) -> RetrievalReport:
      """Aggregate all validation results into structured report"""

  def calculate_status(report: RetrievalReport) -> str:
      """Determine overall status: PASS (≥80% accuracy), FAIL (<50%), PARTIAL (50-80%)"""

  async def write_report_to_file(report: RetrievalReport, output_path: str) -> str:
      """Write JSON report to file; return file path"""

  def format_report_for_console(report: RetrievalReport) -> str:
      """Human-readable text summary for terminal output"""
  ```
- **Acceptance Criteria**:
  - [x] Report JSON is valid and matches schema
  - [x] Status calculation: PASS (≥80%), PARTIAL (50-80%), FAIL (<50%)
  - [x] All metrics calculated and included
  - [x] Query results include top-k with similarity scores and relevance classification
  - [x] Errors list is comprehensive and actionable
- **Dependencies**: T006, T007
- **Effort**: 3 points

#### T009: CLI Interface & Main Entry Point
- **Description**: Create command-line interface for test_retrieval.py with argument parsing
- **File to Create**:
  - `test_retrieval/test_retrieval.py` (main entry point) with CLI argument handling
- **CLI Commands**:
  ```bash
  python test_retrieval.py --validate-all                    # Full validation
  python test_retrieval.py --quick                           # 5 sample queries
  python test_retrieval.py --query-id q001                   # Specific query
  python test_retrieval.py --test-queries custom.json        # Custom queries
  python test_retrieval.py --similarity-threshold 0.70       # Set threshold
  python test_retrieval.py --top-k 10                        # Set top-k results
  python test_retrieval.py --output custom_report.json       # Custom output path
  python test_retrieval.py -v                                # Verbose logging
  python test_retrieval.py --validate-config                 # Config check only
  ```
- **Requirements**:
  - ArgumentParser with clear help messages
  - Default: run full validation with default settings
  - Support all flags from planning document
  - Proper exit codes (0=success, 1=error, 2=partial failure)
- **Acceptance Criteria**:
  - [x] All flags work as documented
  - [x] Help text is clear (python test_retrieval.py --help)
  - [x] Exit codes correct (0=PASS, 1=FAIL, 2=PARTIAL, 127=config error)
  - [x] Error messages are diagnostic
  - [x] Can run from any directory with correct paths
- **Dependencies**: T001-T008
- **Effort**: 2 points

#### T010: Unit & Integration Tests
- **Description**: Create comprehensive test suite for validation pipeline
- **Files to Create**:
  - `test_retrieval/tests/test_config.py` - Configuration validation
  - `test_retrieval/tests/test_retrieval_validator.py` - Core logic tests
  - `test_retrieval/tests/test_report_generator.py` - Report generation tests
  - `test_retrieval/tests/test_integration.py` - End-to-end validation
- **Test Coverage**:
  - Config: environment variables, defaults, validation errors
  - Validator: similarity classification, metadata comparison, chunk ordering
  - Report: schema validation, status calculation, JSON format
  - Integration: full pipeline with mock Qdrant/PostgreSQL
- **Acceptance Criteria**:
  - [x] All modules have unit tests (>80% coverage)
  - [x] Integration test validates end-to-end with mocks
  - [x] Tests run with pytest and pytest-asyncio
  - [x] Mocks provide realistic data (not just null values)
  - [x] Tests pass locally before merge
- **Dependencies**: T001-T009
- **Effort**: 3 points

---

## Phase 2: Testing & Quality Assurance (Future)

### Advanced Testing [P]
- T011: Performance benchmarking (validate <5 min for 1000+ chunks)
- T012: Load testing (concurrent queries, connection pooling)
- T013: Error injection tests (network failures, malformed data)

### Deployment Integration
- T014: GitHub Actions workflow for automated validation
- T015: Vercel post-build hook integration
- T016: Documentation (deployment guide, troubleshooting)

---

## Task Execution Order

### Recommended Execution Sequence

**Sequential (must complete in order)**:
1. T001 (Project structure)
2. T002, T003, T004 (Configuration & clients - can run in parallel)
3. T005 (Test queries)
4. T006 (Core validation)
5. T007 (Similarity evaluation)
6. T008 (Report generation)
7. T009 (CLI interface)
8. T010 (Tests)

**Parallel Execution**:
- T002, T003, T004 can run simultaneously (no dependencies)
- These feed into T006 validation logic

**Total Critical Path**: T001 → T002/T003/T004 (parallel) → T005 → T006 → T007 → T008 → T009 → T010

---

## Success Criteria (All Must Pass)

### Functionality
- [x] All 10 tasks completed
- [x] test_retrieval.py runs without errors
- [x] Pipeline validates Qdrant similarity search results
- [x] Metadata comparison works correctly
- [x] JSON report generated successfully
- [x] All validation metrics included in report

### Performance
- [x] Full validation: < 5 minutes for 1000+ chunks
- [x] Per-query search latency: < 2 seconds
- [x] Report generation: < 30 seconds

### Quality
- [x] No runtime errors or exceptions
- [x] Error handling graceful (continues on individual failures)
- [x] Clear error messages in report
- [x] Code follows project style (PEP 8, async patterns)
- [x] Unit tests pass (>80% coverage)

### Integration
- [x] Works with Feature 006 (embedding pipeline)
- [x] Reuses existing Qdrant and PostgreSQL infrastructure
- [x] CLI is user-friendly

---

## Notes & Constraints

- **Qdrant Availability**: Assumes collection `rag_embedding` exists and is populated by Feature 006
- **PostgreSQL Schema**: Assumes `documents` and `chunks` tables exist with proper relationships
- **Connection Reuse**: Reuse existing asyncpg pool from backend (don't create new connections)
- **Test Data**: Use real queries covering major documentation topics (20+ queries)
- **Idempotent**: Validation can be run multiple times without side effects (read-only operations)
- **Async-First**: All I/O operations must be async (search, metadata retrieval, report generation)

---

## Files Generated Summary

**Core Implementation Files** (10 tasks):
- `test_retrieval/__init__.py`
- `test_retrieval/config.py`
- `test_retrieval/qdrant_client_wrapper.py`
- `test_retrieval/postgres_metadata.py`
- `test_retrieval/query_loader.py`
- `test_retrieval/retrieval_validator.py`
- `test_retrieval/similarity_evaluator.py`
- `test_retrieval/report_generator.py`
- `test_retrieval/test_retrieval.py` (main entry point)
- `test_retrieval/test_queries.json`

**Supporting Files**:
- `test_retrieval/requirements.txt`
- `test_retrieval/README.md`
- `test_retrieval/tests/test_*.py` (4 test files)

**Output**:
- `retrieval_report.json` (generated per run)

**Total LOC Estimate**: ~2,500 lines of production code + tests

---

**Status**: Ready for implementation via `/sp.implement`

Execute tasks in order above to build complete retrieval validation pipeline MVP.
