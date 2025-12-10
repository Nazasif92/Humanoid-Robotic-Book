# Implementation Tasks: Docusaurus Embedding Pipeline

**Feature**: 006-docusaurus-embeddings
**Branch**: `006-docusaurus-embeddings`
**Created**: 2024-01-15
**Status**: Ready for Implementation

---

## Overview

This document breaks down Feature 006 into executable tasks organized by phase. Each task is independently testable and includes acceptance criteria. Parallel tasks are marked with [P].

**MVP Scope**: Phase 1 (Core Pipeline) - complete embedding pipeline from file discovery to Qdrant storage
**Expected Duration**: Phase 1 (10 tasks), Phase 2+ (future iterations)

---

## Phase 1: Core Pipeline Implementation (MVP)

### Setup & Configuration

#### T001: Project Structure & Dependencies
- **Description**: Create embed/ directory structure with Python package setup
- **Files to Create**:
  - `embed/__init__.py` - Package initialization
  - `embed/requirements.txt` - Pinned dependencies (openai, qdrant-client, asyncpg, tiktoken, pyyaml)
  - `embed/README.md` - Usage documentation
- **Acceptance Criteria**:
  - [x] Directory structure matches plan.md
  - [x] requirements.txt includes all dependencies with versions
  - [x] README includes installation and usage instructions
- **Dependencies**: None
- **Effort**: 2 points

#### T002: Configuration & Environment Setup
- **Description**: Create configuration module for environment variables and settings
- **Files to Create**:
  - `embed/config.py` - Settings class with validation
  - Update `.env.example` with embed-specific variables
- **Configuration Variables**:
  - `OPENAI_API_KEY`, `OPENAI_EMBEDDING_MODEL`
  - `QDRANT_URL`, `QDRANT_API_KEY`, `QDRANT_COLLECTION_NAME`
  - `NEON_CONNECTION_STRING`
  - `DOCS_PATH`, `CHUNK_MIN_TOKENS`, `CHUNK_MAX_TOKENS`, `BATCH_SIZE`
- **Acceptance Criteria**:
  - [x] All environment variables validated with pydantic
  - [x] Default values provided where appropriate
  - [x] Configuration can be loaded from .env file
- **Dependencies**: None
- **Effort**: 3 points

### Core Pipeline Implementation

#### T003: File Discovery Module [P]
- **Description**: Implement file discovery to recursively find all .md/.mdx files in docs/
- **File to Create**:
  - `embed/file_discovery.py` with `get_all_urls()` function
- **Function Signature**:
  ```python
  async def get_all_urls(docs_path: str) -> List[Dict[str, str]]:
      """Discover all .md/.mdx files, return list with path, title, url"""
  ```
- **Acceptance Criteria**:
  - [x] Finds all .md and .mdx files recursively
  - [x] Extracts front matter to get title and section
  - [x] Constructs Docusaurus URLs correctly (/docs/...)
  - [x] Returns metadata dict with path, title, url, content_hash
  - [x] Handles errors gracefully (missing files, encoding issues)
- **Dependencies**: None
- **Effort**: 3 points

#### T004: Markdown Parsing & Text Extraction [P]
- **Description**: Parse markdown and extract clean text for embedding
- **File to Create**:
  - `embed/markdown_parser.py` with `extract_text_from_url()` function
- **Function Signature**:
  ```python
  async def extract_text_from_url(file_path: str) -> Tuple[str, Dict]:
      """Parse markdown file, extract text, return content and metadata"""
  ```
- **Requirements**:
  - Extract YAML front matter (title, section, etc.)
  - Remove markdown syntax (##, **, *, etc.)
  - Remove code blocks (backticks, indented)
  - Remove links but keep link text
  - Normalize whitespace
  - Return tuple of (text, metadata)
- **Acceptance Criteria**:
  - [x] Correctly parses YAML front matter
  - [x] Removes all markdown formatting
  - [x] Preserves sentence structure
  - [x] Handles edge cases (empty files, no front matter)
  - [x] Returns metadata dict
- **Dependencies**: None (uses standard library)
- **Effort**: 3 points

#### T005: Token-Based Text Chunking [P]
- **Description**: Chunk text into 300-500 token segments
- **File to Create**:
  - `embed/chunking.py` with `chunk_text()` function
- **Function Signature**:
  ```python
  def chunk_text(text: str, min_tokens: int = 300, max_tokens: int = 500) -> List[str]:
      """Split text into semantic chunks by token count"""
  ```
- **Requirements**:
  - Use tiktoken to count tokens accurately
  - Respect section boundaries (don't split mid-paragraph)
  - Target 300-500 tokens per chunk
  - Return list of chunk strings
- **Acceptance Criteria**:
  - [x] All chunks are 300-500 tokens (±10 tolerance)
  - [x] No mid-sentence splits
  - [x] Handles edge cases (very long sentences, code blocks)
  - [x] Token count is accurate vs. OpenAI model
- **Dependencies**: tiktoken
- **Effort**: 2 points

#### T006: Batch Embedding Generation
- **Description**: Generate embeddings for chunks using OpenAI API with batch processing
- **File to Create**:
  - `embed/embedding_generator.py` with `embed_chunks()` function
- **Function Signature**:
  ```python
  async def embed_chunks(chunks: List[str], batch_size: int = 100) -> List[List[float]]:
      """Batch embed chunks using OpenAI text-embedding-3-small"""
  ```
- **Requirements**:
  - Batch process chunks to minimize API calls
  - Use text-embedding-3-small model (1536 dims)
  - Implement exponential backoff retry logic
  - Return list of 1536-dimensional vectors
- **Acceptance Criteria**:
  - [x] All chunks produce valid 1536-dim embeddings
  - [x] Batch size configurable (default 100)
  - [x] Retry logic with exponential backoff (1s, 2s, 4s)
  - [x] Handles API rate limits gracefully
  - [x] Returns embeddings in same order as input chunks
- **Dependencies**: openai
- **Effort**: 4 points

#### T007: Qdrant Vector Storage
- **Description**: Create Qdrant collection and upsert embedding vectors with metadata
- **File to Create**:
  - `embed/qdrant_storage.py` with `create_collection()` and `save_chunks_to_qdrant()` functions
- **Function Signatures**:
  ```python
  async def create_collection(collection_name: str, vector_size: int = 1536):
      """Initialize Qdrant collection with cosine distance metric"""

  async def save_chunks_to_qdrant(collection_name: str, points: List[Dict]):
      """Upsert embedding points with metadata to Qdrant"""
  ```
- **Requirements**:
  - Create collection if doesn't exist
  - Collection name: `rag_embedding`
  - Vector size: 1536 dimensions
  - Distance metric: cosine
  - Store metadata: doc_id, chunk_index, title, section, url
  - Handle existing points (upsert, not insert)
- **Acceptance Criteria**:
  - [x] Collection created with correct configuration
  - [x] Points upserted successfully
  - [x] Metadata preserved with each point
  - [x] Handles collection already exists error
  - [x] Verifies vectors are stored and searchable
- **Dependencies**: qdrant-client
- **Effort**: 3 points

#### T008: PostgreSQL Metadata Storage
- **Description**: Store chunk metadata in Neon PostgreSQL for querying
- **File to Create**:
  - `embed/postgres_storage.py` with `save_chunks_to_postgres()` function
- **Function Signature**:
  ```python
  async def save_chunks_to_postgres(chunks_data: List[Dict]):
      """Insert/update chunks in PostgreSQL with metadata"""
  ```
- **Requirements**:
  - Insert into existing `documents` and `chunks` tables
  - Store: doc_id, chunk_text, chunk_index, section, token_count, created_at
  - Handle document relationships (FK to documents table)
  - Transaction safety
- **Acceptance Criteria**:
  - [x] Data inserted correctly into documents table
  - [x] Chunks linked to documents with FK
  - [x] Metadata fields preserved
  - [x] Transactions commit successfully
  - [x] Can query by document and chunk index
- **Dependencies**: asyncpg (already setup from RAG chatbot)
- **Effort**: 3 points

### Output & Reporting

#### T009: Manifest Generation & Pipeline Orchestration
- **Description**: Orchestrate full pipeline and generate manifest.json output
- **Files to Create**:
  - `embed/pipeline.py` with main `run_pipeline()` function
  - Manifest JSON output format
- **Function Signature**:
  ```python
  async def run_pipeline(docs_path: str, init_db: bool = False) -> Dict:
      """End-to-end pipeline orchestration"""
  ```
- **Output manifest.json format**:
  ```json
  {
    "status": "success|error",
    "timestamp": "2024-01-15T10:30:00Z",
    "duration_seconds": 120,
    "stats": {
      "files_discovered": 42,
      "files_processed": 42,
      "chunks_created": 1247,
      "errors": 0
    },
    "errors": []
  }
  ```
- **Requirements**:
  - Orchestrate T003-T008 functions in order
  - Track timing and statistics
  - Handle errors and continue (resilience)
  - Export manifest.json
  - Return success/error status
- **Acceptance Criteria**:
  - [x] All pipeline stages execute in correct order
  - [x] Statistics tracked accurately
  - [x] Manifest.json created with correct format
  - [x] Error handling continues processing despite individual failures
  - [x] Total duration < 10 minutes for 40+ documents
- **Dependencies**: All previous tasks (T003-T008)
- **Effort**: 4 points

#### T010: CLI Interface & Main Entry Point
- **Description**: Create command-line interface for embed.py with argument parsing
- **Files to Create**:
  - `embed/embed.py` (main entry point) with CLI argument handling
- **CLI Commands**:
  ```bash
  python embed.py --docs-path ./docs --init-db              # Full re-index
  python embed.py --docs-path ./docs                         # Index without init
  python embed.py --docs-path ./docs --summary               # Stats only
  python embed.py --validate                                 # Config validation
  ```
- **Requirements**:
  - ArgumentParser with clear help messages
  - --docs-path argument (required, default ./docs)
  - --init-db flag (optional, initialize database)
  - --incremental flag (future: only changed files)
  - --summary flag (optional: print stats without embedding)
  - --validate flag (optional: check configuration)
  - Proper exit codes (0=success, 1=error)
- **Acceptance Criteria**:
  - [x] All flags work as documented
  - [x] Help text is clear (python embed.py --help)
  - [x] Exit codes correct
  - [x] Error messages are diagnostic
  - [x] Can run from any directory with correct paths
- **Dependencies**: T009 (pipeline orchestration)
- **Effort**: 2 points

---

## Phase 2: Testing & Quality Assurance (Future)

### Unit Tests [P]
- T011: Test file discovery (test_file_discovery.py)
- T012: Test markdown parsing (test_markdown_parser.py)
- T013: Test text chunking (test_chunking.py)
- T014: Test embedding generation (test_embedding_generator.py)
- T015: Test Qdrant storage (test_qdrant_storage.py)
- T016: Test PostgreSQL storage (test_postgres_storage.py)

### Integration Tests
- T017: End-to-end pipeline test with sample docs
- T018: Manifest validation test
- T019: CLI interface test

### Performance Tests
- T020: Benchmark 40+ document indexing
- T021: Verify <10 min full indexing, <30 sec incremental

---

## Phase 3: Incremental Indexing (Future)

- T022: Implement manifest.json tracking
- T023: Implement hash-based change detection
- T024: Implement delete handling
- T025: Implement --incremental flag

---

## Phase 4: Deployment Integration (Future)

- T026: Vercel post-build hook
- T027: GitHub Actions workflow
- T028: Deployment documentation

---

## Task Execution Order

### Recommended Execution Sequence

**Sequential (must complete in order)**:
1. T001 (Project structure)
2. T002 (Configuration)
3. T003-T005 (Run in parallel [P])
4. T006 (Embedding - depends on T005)
5. T007-T008 (Run in parallel [P] - both independent)
6. T009 (Orchestration - depends on T003-T008)
7. T010 (CLI - depends on T009)

**Parallel Execution**:
- T003, T004, T005 can run simultaneously
- T007, T008 can run simultaneously

**Total Critical Path**: T001 → T002 → T003/T004/T005 → T006 → T007/T008 → T009 → T010

---

## Success Criteria (All Must Pass)

### Functionality
- [ ] All 10 tasks completed
- [ ] Embed.py runs without errors
- [ ] Pipeline produces manifest.json
- [ ] All embeddings stored in Qdrant
- [ ] All metadata stored in PostgreSQL

### Performance
- [ ] Full indexing: < 10 minutes for 40+ documents
- [ ] ~1,200+ chunks created
- [ ] All chunks 300-500 tokens

### Quality
- [ ] No runtime errors
- [ ] Error handling graceful (continues on individual file errors)
- [ ] Clear error messages in manifest
- [ ] Code follows project style (PEP 8, async patterns)

### Testing (Phase 2)
- [ ] Unit tests for each module
- [ ] Integration test for full pipeline
- [ ] CLI tests all flags

---

## Notes & Constraints

- **Database Schema**: Assumes documents and chunks tables exist (created by backend Phase 1)
- **API Keys**: OpenAI, Qdrant, Neon credentials required before running
- **Python Version**: 3.11+ (async/await patterns)
- **Reuse**: Integrate with existing backend services (neon_client, qdrant_client patterns)
- **Idempotent**: Pipeline can be run multiple times safely (upsert, not insert)

---

## Files Generated Summary

**Core Implementation Files** (10 tasks):
- `embed/__init__.py`
- `embed/config.py`
- `embed/file_discovery.py`
- `embed/markdown_parser.py`
- `embed/chunking.py`
- `embed/embedding_generator.py`
- `embed/qdrant_storage.py`
- `embed/postgres_storage.py`
- `embed/pipeline.py`
- `embed/embed.py` (main entry point)

**Supporting Files**:
- `embed/requirements.txt`
- `embed/README.md`
- `manifest.json` (output)

**Total LOC Estimate**: ~2,000 lines of production code + tests

---

**Status**: Ready for implementation via `/sp.implement`

Execute tasks in order above to build complete embedding pipeline MVP.
