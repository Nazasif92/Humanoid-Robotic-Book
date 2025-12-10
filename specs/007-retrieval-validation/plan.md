# Implementation Plan: Retrieval Pipeline Validation

**Branch**: `007-retrieval-validation` | **Date**: 2024-01-15 | **Spec**: [specs/007-retrieval-validation/spec.md](spec.md)
**Input**: Feature specification + planning requirements for validation script and JSON reporting

## Summary

Create a retrieval pipeline validation framework that verifies Qdrant vector similarity search returns semantically relevant chunks with accurate metadata. The system will load test queries, execute similarity searches (top-k=5-10), validate metadata integrity (URLs, paths, chunk indices), evaluate relevance using cosine similarity thresholds, and generate a structured JSON report (retrieval_report.json) documenting accuracy, failures, and missing entries. Deliverable: `test_retrieval.py` script integrated with Feature 006 embedding pipeline for end-to-end quality assurance.

## Technical Context

**Language/Version**: Python 3.11+ (async/await patterns, type hints)

**Primary Dependencies**:
- `qdrant-client[http]` - Vector database client with async support
- `asyncpg` - Async PostgreSQL driver (already in backend)
- `openai` - Embedding generation for test query vectors
- `numpy` - Vector operations (cosine similarity)
- `pydantic` - Data validation for report schema
- `pytest-asyncio` - Async test execution

**Storage**:
- Qdrant Cloud - Vector storage with metadata payloads
- PostgreSQL (Neon) - Chunk metadata (documents, chunks tables)
- Local filesystem - retrieval_report.json output

**Testing**: pytest + pytest-asyncio for async test cases; mock Qdrant/PostgreSQL for unit tests

**Target Platform**: Linux/macOS/Windows CLI tool; runnable locally or in CI/CD pipelines (GitHub Actions, Vercel)

**Project Type**: Single Python CLI validation tool with async architecture

**Performance Goals**:
- Validate 1000+ chunks in < 5 minutes
- Similarity search latency < 2 seconds per query
- Report generation < 30 seconds
- Memory usage < 500MB (avoid loading all vectors into memory)

**Constraints**:
- Qdrant API rate limiting (handle gracefully with backoff)
- PostgreSQL connection pooling (reuse existing 1-20 connection pool)
- Embedding cost (batch test queries to minimize OpenAI API calls)
- JSON report must fit in memory (streaming not required for <100k chunks)

**Scale/Scope**:
- ~1200+ documentation chunks from Feature 006
- 20+ test queries (covering diverse topics)
- Top-k results = 5-10 per query
- Metadata validation for all retrieved chunks

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on **Humanoid Robotic Book Constitution**:

| Principle | Check | Status | Notes |
|-----------|-------|--------|-------|
| **I. Accuracy & Correctness** | Validation logic mirrors actual RAG pipeline behavior; metrics are verifiable against real systems | ✅ PASS | Cosine similarity, metadata comparison, error detection all testable |
| **II. Beginner-Friendly Language** | Code examples in README; error messages are diagnostic; report format is readable | ✅ PASS | JSON report human-readable; CLI help text clear; examples provided |
| **III. Consistent Style & Structure** | Follows existing backend patterns (async, Pydantic, error handling, logging) | ✅ PASS | Mirrors backend architecture (app/rag_pipeline.py, neon_client.py patterns) |
| **IV. Example-Driven Explanations** | test_retrieval.py script is copy-paste runnable; includes CLI examples | ✅ PASS | Script includes docstrings, example queries, expected output format |
| **V. Quality Review & Fact-Check** | All validation logic testable; JSON schema validated; metrics defined | ✅ PASS | Tests for accuracy calculation, metadata validation, error detection |

**Gate Status**: ✅ **PASS** - Feature aligns with all 5 constitution principles

## Project Structure

### Documentation (this feature)

```text
specs/007-retrieval-validation/
├── spec.md                          # Feature specification (user stories, requirements, success criteria)
├── plan.md                          # This file (architecture and technical decisions)
├── checklists/
│   └── requirements.md              # Quality validation checklist
└── (future phases)
    ├── research.md                  # Phase 0: research on validation approaches
    ├── data-model.md                # Phase 1: detailed entity definitions
    └── quickstart.md                # Phase 1: getting started guide
```

### Source Code (repository root)

```text
test_retrieval/                      # New directory for validation tool
├── test_retrieval.py               # Main validation script (single entry point)
├── requirements.txt                # Dependencies (qdrant-client, asyncpg, numpy, pydantic)
├── README.md                       # Usage guide and examples
├── config.py                       # Configuration & environment variables
├── retrieval_validator.py          # Core validation logic class
├── report_generator.py             # JSON report generation
├── test_queries.json               # Sample test queries (20+ diverse topics)
└── tests/
    ├── test_config.py              # Configuration validation
    ├── test_retrieval_validator.py # Unit tests for validation logic
    ├── test_report_generator.py    # Unit tests for report generation
    └── test_integration.py         # End-to-end validation tests

output/
└── retrieval_report.json           # Generated validation report (gitignored)

specs/007-retrieval-validation/
├── spec.md
├── plan.md
└── checklists/requirements.md
```

**Structure Decision**: Single `test_retrieval.py` script (main entry point) with modular support files for clarity and testability. Mirrors Feature 006 (embed.py) single-script design for deployment simplicity. Separates concerns (config, validation logic, reporting) into reusable modules while keeping CLI interface minimal and focused.

## Complexity Tracking

*No Constitution Check violations detected. Design choices are justified by simplicity and consistency with existing architecture.*

---

## Architectural Decisions

### Decision 1: Qdrant Client Connection Strategy

**Choice**: Create long-lived async QdrantClient that reuses connection across all queries

**Rationale**:
- Single connection avoids per-query initialization overhead
- Async client enables batching and pipelined requests
- Matches existing backend pattern (app/qdrant_client.py)
- Connection pooling handled by qdrant-client library

**Implementation**:
```python
from qdrant_client.async_client import AsyncClient

client = AsyncClient(
    url=settings.QDRANT_URL,
    api_key=settings.QDRANT_API_KEY,
    timeout=30.0
)
# Reuse across all validation operations
```

**Alternatives Considered**:
- Per-query connections: Simpler but inefficient; violates consistency with backend
- Sync client: Blocks event loop; would require process-based concurrency

---

### Decision 2: Test Query Generation and Relevance Evaluation

**Choice**: Use curated set of 20+ test queries with keyword-based relevance labels + cosine similarity threshold

**Rationale**:
- Keyword labels enable coverage of all major documentation topics
- Cosine similarity (from Qdrant) provides quantitative relevance score
- Threshold-based evaluation (≥0.75) aligns with feature specification SC-001
- Avoids expensive LLM-based evaluation; uses deterministic metrics

**Implementation**:
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

**Alternatives Considered**:
- Manual expert evaluation: Too time-consuming; not scalable
- LLM-based evaluation: Expensive and adds API dependency
- Exact match only: Too strict; fails on synonyms and paraphrases

---

### Decision 3: Metadata Integrity Validation

**Choice**: Compare Qdrant metadata payloads against PostgreSQL records with 5-field validation

**Rationale**:
- 5 critical fields (url, path, title, section, chunk_index) ensure usability
- Qdrant-vs-PostgreSQL comparison detects consistency issues early
- Field-by-field validation enables precise error reporting
- Matches Feature 006 data model exactly

**Validation Fields**:
1. **url** - Constructed Docusaurus URL (e.g., `/docs/ros2-basics`)
2. **path** - Source file path relative to docs/ (e.g., `intro/basics.md`)
3. **title** - Document title from front matter (e.g., "ROS 2 Basics")
4. **section** - Section name from metadata (e.g., "Introduction")
5. **chunk_index** - Sequence order within document (0-based integer)

**Implementation**:
```python
async def validate_metadata(chunk_id: str, qdrant_metadata: dict, pg_record: dict) -> ValidationResult:
    """Compare 5 critical metadata fields"""
    errors = []
    if qdrant_metadata.get('url') != pg_record.get('url'):
        errors.append(f"URL mismatch: {qdrant_metadata['url']} vs {pg_record['url']}")
    if qdrant_metadata.get('chunk_index') != pg_record.get('chunk_index'):
        errors.append(f"Index mismatch: {qdrant_metadata['chunk_index']} vs {pg_record['chunk_index']}")
    # ... validate path, title, section ...
    return ValidationResult(chunk_id=chunk_id, is_valid=len(errors)==0, errors=errors)
```

**Alternatives Considered**:
- Qdrant-only validation: Misses synchronization issues between systems
- PostgreSQL-only validation: Doesn't verify vector storage integrity
- Partial field validation: Risks missing critical errors

---

### Decision 4: Cosine Similarity Calculation and Thresholding

**Choice**: Use Qdrant's similarity scores directly; set threshold ≥0.75 for "relevant" classification

**Rationale**:
- Qdrant pre-computes cosine similarity; no need for local vector loading
- Threshold 0.75 aligns with specification SC-001 (80% top-3 relevant)
- Allows dynamic threshold configuration (tunable by user)
- Matches embedding model capabilities (text-embedding-3-small is high-quality)

**Threshold Levels**:
- ≥0.80 = Highly relevant (gold standard)
- 0.70-0.79 = Relevant (acceptable)
- <0.70 = Low relevance (failure)

**Implementation**:
```python
async def evaluate_search_quality(query: str, results: List[SearchResult]) -> RelevanceMetrics:
    """Classify results by similarity threshold"""
    highly_relevant = sum(1 for r in results if r.similarity_score >= 0.80)
    relevant = sum(1 for r in results if 0.70 <= r.similarity_score < 0.80)
    low_relevance = sum(1 for r in results if r.similarity_score < 0.70)

    return RelevanceMetrics(
        query=query,
        highly_relevant_count=highly_relevant,
        relevant_count=relevant,
        low_relevance_count=low_relevance,
        relevance_ratio=highly_relevant / len(results) if results else 0
    )
```

**Alternatives Considered**:
- Local cosine similarity calculation: Redundant; requires loading all vectors
- Hard threshold without gradation: Less diagnostic; harder to debug
- LLM-based relevance judgment: Expensive; non-deterministic

---

### Decision 5: JSON Report Schema and Structure

**Choice**: Hierarchical report with summary → metrics → query results → error log

**Rationale**:
- Top-level summary enables quick pass/fail assessment
- Per-query breakdown enables debugging specific failures
- Metrics section provides quantitative quality indicators
- Error log captures actionable diagnostics for remediation
- Schema enables programmatic parsing and CI/CD integration

**Report Structure**:
```json
{
  "status": "PASS|FAIL|PARTIAL",
  "timestamp": "2024-01-15T10:30:00Z",
  "summary": {
    "total_queries_tested": 20,
    "queries_passed": 16,
    "queries_failed": 4,
    "overall_accuracy_percent": 80.0,
    "validation_duration_seconds": 45
  },
  "metrics": {
    "metadata_accuracy_percent": 100.0,
    "url_accuracy_percent": 100.0,
    "chunk_ordering_accuracy_percent": 98.5,
    "avg_search_latency_ms": 1250,
    "total_chunks_validated": 1247
  },
  "query_results": [
    {
      "query_id": "q001",
      "query_text": "What is ROS 2?",
      "status": "PASS",
      "top_k_results": [
        {
          "rank": 1,
          "chunk_id": "doc_42_chunk_0",
          "similarity_score": 0.87,
          "title": "ROS 2 Basics",
          "url": "/docs/ros2-basics",
          "relevance": "HIGHLY_RELEVANT",
          "metadata_valid": true
        }
      ],
      "accuracy_percent": 100.0
    }
  ],
  "errors": [
    {
      "type": "MISSING_CHUNK",
      "chunk_id": "doc_99_chunk_5",
      "description": "Chunk exists in PostgreSQL but not in Qdrant"
    },
    {
      "type": "METADATA_MISMATCH",
      "chunk_id": "doc_42_chunk_0",
      "field": "url",
      "qdrant_value": "/docs/ros2-basics",
      "postgres_value": "/docs/ros-2-basics",
      "description": "URL mismatch between Qdrant and PostgreSQL"
    }
  ]
}
```

**Alternatives Considered**:
- Flat report: Easier to generate but harder to navigate and debug
- Multiple file outputs: Fragmented; harder to track as single unit
- CSV report: Not human-readable; loses hierarchical context

---

## Data Model

### Entities

**1. TestQuery**
- `query_id`: Unique identifier (e.g., "q001")
- `text`: Query string (e.g., "What is ROS 2?")
- `expected_keywords`: List of keywords for validation (e.g., ["ROS", "robotics"])
- `relevance_threshold`: Minimum similarity score (0.0-1.0, default 0.75)
- `expected_top_k`: Expected number of relevant results (default 3)

**2. SearchResult**
- `chunk_id`: Unique identifier from Qdrant
- `similarity_score`: Cosine similarity (0.0-1.0)
- `chunk_text`: Full text of chunk
- `title`: Document title
- `section`: Section name
- `url`: Docusaurus URL
- `path`: Source file path
- `chunk_index`: Sequence within document
- `relevance_classification`: "HIGHLY_RELEVANT" | "RELEVANT" | "LOW_RELEVANCE"

**3. ValidationResult**
- `chunk_id`: Chunk being validated
- `is_valid`: Boolean (all metadata fields match)
- `errors`: List of mismatches
- `qdrant_metadata`: Metadata from Qdrant
- `postgres_metadata`: Metadata from PostgreSQL

**4. RetrievalReport**
- `status`: "PASS" | "FAIL" | "PARTIAL"
- `timestamp`: ISO 8601 datetime
- `summary`: ReportSummary object
- `metrics`: RetrievalMetrics object
- `query_results`: List[QueryResult]
- `errors`: List[ValidationError]
- `validation_duration_seconds`: Total elapsed time

**5. ReportSummary**
- `total_queries_tested`: Count
- `queries_passed`: Count
- `queries_failed`: Count
- `overall_accuracy_percent`: Calculated metric
- `queries_with_errors`: List of query_ids

**6. RetrievalMetrics**
- `metadata_accuracy_percent`: (chunks with valid metadata / total) * 100
- `url_accuracy_percent`: (URLs matching source paths / total) * 100
- `chunk_ordering_accuracy_percent`: (correct chunk sequences / total) * 100
- `avg_search_latency_ms`: Mean latency across all queries
- `total_chunks_validated`: Count of unique chunks checked

## API Contracts

### CLI Interface (test_retrieval.py)

```bash
# Full validation with test queries
python test_retrieval.py --docs-path ./docs --validate-all

# Quick validation (5 sample queries only)
python test_retrieval.py --quick

# Validate specific query by ID
python test_retrieval.py --query-id q001

# Load custom test queries from file
python test_retrieval.py --test-queries custom_queries.json

# Set similarity threshold
python test_retrieval.py --similarity-threshold 0.70

# Set top-k for retrieval
python test_retrieval.py --top-k 10

# Output report path
python test_retrieval.py --output report_custom.json

# Verbose logging
python test_retrieval.py -v

# Validate configuration only
python test_retrieval.py --validate-config
```

### Environment Variables

```bash
# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key
QDRANT_COLLECTION_NAME=rag_embedding

# Neon PostgreSQL
NEON_CONNECTION_STRING=postgresql://user:password@host/db

# OpenAI (for embedding test queries)
OPENAI_API_KEY=sk-...
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Validation settings
RETRIEVAL_SIMILARITY_THRESHOLD=0.75
RETRIEVAL_TOP_K=5
RETRIEVAL_BATCH_SIZE=20
```

### Output Contract (retrieval_report.json)

```json
{
  "status": "PASS|FAIL|PARTIAL",
  "timestamp": "ISO8601",
  "summary": { ... },
  "metrics": { ... },
  "query_results": [ ... ],
  "errors": [ ... ]
}
```

Exit codes:
- `0` = PASS (all queries returned relevant results)
- `1` = FAIL (significant validation failures)
- `2` = PARTIAL (some queries failed, overall system functional)
- `127` = Configuration error

---

## Implementation Phases

### Phase 1: Core Validation Pipeline (MVP)

**Tasks**:
1. Configuration module (config.py) with environment validation
2. Qdrant client initialization and connection management
3. PostgreSQL client reuse from backend (asyncpg)
4. Core retrieval validation logic (search + metadata comparison)
5. Test query dataset (test_queries.json with 20+ queries)
6. JSON report generation (retrieval_report.json schema)
7. CLI interface with argument parsing (test_retrieval.py)
8. Unit tests for each component
9. Integration test (end-to-end validation)
10. README with usage examples

**Deliverables**:
- test_retrieval.py (single entry point)
- retrieval_validator.py (core logic)
- report_generator.py (JSON output)
- test_queries.json (test data)
- Unit + integration tests
- README.md with CLI examples

**Estimated**: 10 tasks, ~50 story points

### Phase 2: Advanced Features (Future)

- Incremental validation (only changed chunks)
- Custom metric definitions
- Integration with CI/CD (GitHub Actions, Vercel)
- Performance benchmarking
- Visual report generation (HTML)

### Phase 3: Production Deployment (Future)

- Cloud-based validation service
- Scheduled validation runs
- Alerting on quality degradation
- Metrics dashboard

---

## Success Criteria for Implementation

1. **Functionality**: test_retrieval.py runs without errors; generates valid retrieval_report.json
2. **Quality**: ≥80% top-3 search results semantically relevant; 100% metadata accuracy
3. **Performance**: Validates 1000+ chunks in <5 minutes; per-query latency <2s
4. **Reliability**: Handles errors gracefully (timeouts, missing data, API failures)
5. **Usability**: CLI help text clear; error messages diagnostic; examples in README
6. **Testing**: Unit tests for config, validation, reporting; integration test for full pipeline
7. **Documentation**: README includes setup, CLI examples, troubleshooting
8. **Integration**: Works with existing Qdrant + PostgreSQL infrastructure from Features 1-6

---

## Risks and Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|-----------|
| Qdrant API rate limiting during validation | Medium | Delays report generation | Implement exponential backoff; batch queries (max 20 per second) |
| PostgreSQL connection exhaustion | Low | Validation hangs | Reuse existing connection pool (1-20 connections); timeout after 30s |
| Test query keyword labels become outdated | Medium | False accuracy metrics | Version test_queries.json; update quarterly when docs change |
| Cosine threshold (0.75) too strict | Medium | False negatives | Make threshold configurable; analyze failure patterns |

---

## Dependencies and Assumptions

**Dependencies**:
- Feature 006 (embedding pipeline) must be complete
- Qdrant collection `rag_embedding` must exist and be populated
- PostgreSQL `documents` and `chunks` tables must exist with metadata
- OpenAI API for embedding test queries

**Assumptions**:
- Test queries cover major documentation topics adequately
- Cosine similarity ≥0.75 correlates with semantic relevance
- Metadata inconsistencies are rare but important to detect
- Validation is run after embedding pipeline completes (not real-time monitoring)

