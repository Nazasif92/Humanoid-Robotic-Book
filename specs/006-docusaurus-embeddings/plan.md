# Implementation Plan: Docusaurus Embedding Pipeline

**Branch**: `006-docusaurus-embeddings` | **Date**: 2024-01-15 | **Spec**: [specs/006-docusaurus-embeddings/spec.md](spec.md)
**Input**: Feature specification from `/specs/006-docusaurus-embeddings/spec.md`

## Summary

Create an end-to-end embedding pipeline that discovers all .md/.mdx files in a Docusaurus documentation project, extracts and normalizes content, chunks text by token count (300-500 tokens), generates semantic embeddings using OpenAI, stores them in Qdrant, and persists metadata in PostgreSQL. The pipeline supports both full re-indexing and incremental updates, provides a Python CLI script (embed.py) for local execution, and integrates with Vercel deployment workflows for automated production indexing.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**:
- OpenAI API (text-embedding-3-small)
- Qdrant client (vector database)
- asyncpg (PostgreSQL async)
- tiktoken (token counting)
- PyYAML (front matter parsing)
- pathlib (file discovery)

**Storage**:
- PostgreSQL (chunk metadata, documents, timestamps)
- Qdrant Cloud (1536-dim vector embeddings)
- Local filesystem (docs/ directory)

**Testing**: pytest, pytest-asyncio, mock for external services
**Target Platform**: Linux/macOS/Windows (CLI tool), Vercel deployment (CI/CD integration)
**Project Type**: Single Python CLI tool with database backing
**Performance Goals**:
- Full re-indexing: < 10 minutes for 40+ documents
- Incremental update: < 30 seconds for single page change
- Semantic search precision: > 80% for top-3 results
- Token-based chunking: 300-500 tokens per chunk

**Constraints**:
- OpenAI API quota management (batch processing to minimize calls)
- Qdrant cluster availability (graceful fallback)
- Metadata preservation for source attribution
- Deduplication to avoid redundant embeddings

**Scale/Scope**:
- 40+ Docusaurus documentation pages
- ~1,200+ content chunks
- Support for .md and .mdx files
- Production deployment on Vercel

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on **Humanoid Robotic Book Constitution**:

| Principle | Check | Status | Notes |
|-----------|-------|--------|-------|
| **I. Accuracy & Correctness** | Code examples testable and verified | ✅ PASS | All code in embed.py will be tested; examples match actual implementation |
| **II. Beginner-Friendly Language** | Documentation uses simple, clear English | ✅ PASS | Code comments explain "why" not "what"; README targets operators not experts |
| **III. Consistent Style & Structure** | Follows project patterns (async, error handling, logging) | ✅ PASS | Aligns with existing Phase 1-7 RAG chatbot architecture |
| **IV. Example-Driven Explanations** | Working code samples with expected output | ✅ PASS | embed.py script is copy-paste runnable; includes CLI examples |
| **V. Quality Review & Fact-Check** | Tests pass; facts verified against OpenAI/Qdrant docs | ✅ PASS | Will include unit + integration tests; API calls validated |

**Gate Status**: ✅ **PASS** - Feature aligns with all constitution principles

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

```text
embed/
├── embed.py                    # Main CLI script (single entry point)
├── requirements.txt            # Python dependencies
├── README.md                   # Usage guide and API reference
├── manifest.json              # Output: processed files inventory
└── tests/
    ├── test_file_discovery.py  # Unit tests for file loading
    ├── test_parsing.py         # Unit tests for markdown parsing
    ├── test_chunking.py        # Unit tests for token-based chunking
    ├── test_embedding.py       # Integration tests with OpenAI mock
    ├── test_qdrant.py          # Integration tests with Qdrant mock
    └── test_integration.py     # End-to-end pipeline tests

docs/                          # Docusaurus documentation (input)
├── intro.md
├── chapter1.md
├── chapter2.md
└── ... (40+ markdown files)

backend/                       # Existing RAG chatbot (uses embeddings)
├── app/
│   └── ... (existing code)
└── ...
```

**Structure Decision**: Single-file Python CLI tool (`embed.py`) with modular functions for each pipeline stage. Follows existing backend architecture patterns (async, error handling, logging). Integrates with existing database connections and Qdrant setup from RAG chatbot (Phase 1-3).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

*No violations detected*

---

## Architectural Decisions

### Decision 1: Single Python Script vs. Modular Package

**Choice**: Single `embed.py` script with modular internal functions

**Rationale**:
- Simpler deployment (copy one file)
- No package installation overhead
- Easy to integrate with GitHub Actions
- Matches Docusaurus deployment simplicity
- Can be integrated into Vercel build process as post-deployment hook

**Alternatives Considered**:
- Full Python package: Better for reusability but adds distribution complexity
- Separate service: Overcomplicated for one-off indexing task

---

### Decision 2: Token-Based Chunking Strategy

**Choice**: Fixed 300-500 token chunks respecting section boundaries

**Rationale**:
- Balanced chunk size for semantic coherence
- Prevents mid-sentence splits
- Consistent with OpenAI embedding model capabilities
- Reduces API calls vs. per-paragraph chunking
- Supports both full and incremental re-indexing

**Alternatives Considered**:
- Fixed word count: Inconsistent with actual token usage
- Per-paragraph: Produces variable chunk sizes
- Recursive splitting: Complex, harder to maintain

---

### Decision 3: OpenAI text-embedding-3-small

**Choice**: Use OpenAI `text-embedding-3-small` model (1536 dimensions)

**Rationale**:
- Consistent with Phase 1-7 RAG chatbot implementation
- High quality embeddings with good performance
- Smaller model reduces API costs vs. text-embedding-3-large
- 1536 dimensions well-supported by Qdrant
- Proven in production with existing RAG system

**Alternatives Considered**:
- Cohere Embed: Different API but similar quality; would require switching from OpenAI
- Open-source (sentence-transformers): Lower quality, requires local compute
- GPT-4 embeddings: Overkill for documentation; higher cost

---

### Decision 4: Incremental Indexing via Hash-Based Detection

**Choice**: Track file content hashes and modification times to detect changes

**Rationale**:
- Reliable change detection (hash prevents false positives)
- Minimal storage overhead (manifest.json)
- Fast detection (no full file re-parse)
- Supports deletions (missing files removed from Qdrant)
- No external state required (reproducible)

**Alternatives Considered**:
- Git-based tracking: Requires git repo; not universally available
- Timestamp only: Unreliable (can be modified)
- Always full re-index: Wasteful; violates P2 story efficiency goal

---

### Decision 5: PostgreSQL Metadata Store

**Choice**: Reuse existing Neon PostgreSQL from RAG chatbot for chunk metadata

**Rationale**:
- Already set up and proven in production (Phases 1-3)
- Supports relational queries (find chunks by doc, section, date)
- Integrates with existing RAG pipeline (chunks table)
- Transaction support for consistency
- Backup/restore capability

**Alternatives Considered**:
- JSON files: Less queryable; no ACID guarantees
- Qdrant payload only: Limited metadata structure; harder to query
- Separate database: Adds operational complexity

---

## Data Model

### Entities

1. **Documentation File**
   - Path (relative to docs/)
   - Title (from front matter)
   - Section (from front matter)
   - URL (constructed from path)
   - Content hash
   - Last modified time
   - Processing status (pending, indexed, error)

2. **Content Chunk**
   - Chunk ID (unique)
   - Document ID (FK)
   - Chunk text (300-500 tokens)
   - Chunk index (position in document)
   - Token count (actual)
   - Embedding (1536 dimensions)
   - Created at timestamp

3. **Metadata Manifest** (manifest.json)
   - Version
   - Index timestamp
   - File count
   - Chunk count
   - Errors (if any)
   - Index duration seconds

---

## API Contracts

### CLI Interface (embed.py)

```bash
# Full re-index
python embed.py --docs-path ./docs --init-db

# Incremental update
python embed.py --docs-path ./docs --incremental

# Output summary
python embed.py --docs-path ./docs --summary

# Validate configuration
python embed.py --validate
```

### Environment Variables

```bash
OPENAI_API_KEY=sk-...
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

QDRANT_URL=https://...
QDRANT_API_KEY=...
QDRANT_COLLECTION_NAME=rag_embedding

NEON_CONNECTION_STRING=postgresql://...

DOCS_PATH=./docs
CHUNK_MIN_TOKENS=300
CHUNK_MAX_TOKENS=500
BATCH_SIZE=100
```

### Output Contract

```json
{
  "status": "success|error",
  "timestamp": "2024-01-15T10:30:00Z",
  "duration_seconds": 120,
  "stats": {
    "files_discovered": 42,
    "files_processed": 42,
    "files_skipped": 0,
    "chunks_created": 1247,
    "errors": 0
  },
  "errors": []
}
```

---

## Implementation Phases

### Phase 1: Core Pipeline (MVP)
- File discovery and loading
- Markdown parsing and text extraction
- Token-based chunking
- Batch embedding generation
- Qdrant collection creation and upsert
- PostgreSQL metadata storage
- CLI interface with --init-db flag
- Error handling and logging
- Unit tests for each component

### Phase 2: Incremental Indexing
- Manifest.json tracking
- File hash-based change detection
- Delete handling (remove from Qdrant/Neon)
- --incremental flag support
- Integration tests

### Phase 3: Deployment Integration
- Vercel post-build hook
- GitHub Actions workflow
- Automated production indexing
- Deployment documentation

---

## Success Criteria for Implementation

1. **Completeness**: All .md/.mdx files (100%) indexed with correct metadata
2. **Performance**: Full re-indexing < 10 min; incremental < 30 sec
3. **Quality**: > 80% semantic search precision; 0 runtime errors
4. **Maintainability**: Code includes comments; tests pass; logs are diagnostic
5. **Integration**: Works with existing Qdrant collection and Neon database
6. **Automation**: Runs without manual intervention; suitable for CI/CD pipelines
