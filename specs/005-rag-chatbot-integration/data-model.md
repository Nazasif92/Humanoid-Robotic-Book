# Data Model: RAG Chatbot

**Date**: 2025-12-07 | **Feature**: 005-rag-chatbot-integration

## Entity Definitions

### 1. Document

Represents a markdown file from the Docusaurus `/docs` folder.

**Table**: `documents` (PostgreSQL)

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | SERIAL | PRIMARY KEY | Unique document identifier |
| title | TEXT | NOT NULL | Extracted from markdown frontmatter (e.g., "Module 1: ROS 2") |
| section | TEXT | NOT NULL | Folder path in `/docs` (e.g., "chapters", "appendices") |
| url | TEXT | NOT NULL | Docusaurus routing path (e.g., "/docs/chapters/ros2-nervous-system") |
| content | TEXT | NOT NULL | Full markdown file content |
| created_at | TIMESTAMP | DEFAULT NOW() | When document was indexed |
| updated_at | TIMESTAMP | DEFAULT NOW() ON UPDATE CURRENT_TIMESTAMP | When document was last refreshed |

**Uniqueness**: `UNIQUE(url)` — One document per Docusaurus URL

**State Transitions**:
- **NEW** → **INDEXED**: Document loaded, parsed, chunks created
- **INDEXED** → **REFRESHED**: Document re-ingested (e.g., during scheduled 24-hour refresh)

**Validation Rules**:
- `title` must not be empty
- `url` must start with `/docs/` and follow Docusaurus routing conventions
- `section` must match folder structure in repository

---

### 2. Chunk

A segment of a document, split for embedding and retrieval. Chunks are the atomic unit for RAG.

**Table**: `chunks` (PostgreSQL metadata + Qdrant vectors)

| Field | Type | Storage | Constraints | Description |
|-------|------|---------|-------------|-------------|
| id | SERIAL | PostgreSQL | PRIMARY KEY | Unique chunk identifier |
| doc_id | INTEGER | PostgreSQL | FK → documents.id | Parent document reference |
| chunk_text | TEXT | PostgreSQL | NOT NULL | Chunk content (300-500 tokens, verbatim from document) |
| chunk_index | INTEGER | PostgreSQL | NOT NULL | Position in parent document (0-based) |
| embedding | VECTOR[1536] | Qdrant | NOT NULL | OpenAI embedding vector (stored in Qdrant, referenced by ID) |
| section | TEXT | PostgreSQL | NOT NULL | Inherited from document for filtering/sorting |
| created_at | TIMESTAMP | PostgreSQL | DEFAULT NOW() | When chunk was created |

**Qdrant Collection**: `humanoid_docs`
- **Vector size**: 1536 (text-embedding-3-small output dimension)
- **Distance metric**: Cosine similarity (for semantic search)
- **Payload**: `{ doc_id, chunk_index, section }` (metadata for lookup in PostgreSQL)

**Uniqueness**: `UNIQUE(doc_id, chunk_index)` — One chunk per document position

**State Transitions**:
- **CREATED** → **INDEXED**: Chunk embedded and stored in Qdrant
- **INDEXED** → **REFRESHED**: Re-embedded if parent document changes

**Validation Rules**:
- `chunk_text` must be 50-1500 tokens (using tiktoken count)
- `chunk_index` must be sequential within document (no gaps)
- `embedding` must be a valid 1536-dimensional float vector

**Lifecycle**:
1. Document is parsed into chunks
2. Each chunk is embedded using OpenAI API
3. Vector stored in Qdrant; metadata stored in PostgreSQL
4. On document update, old chunks deleted, new chunks created

---

### 3. ChatLog

Records a question-answer pair for audit, analytics, and debugging.

**Table**: `chat_logs` (PostgreSQL)

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | SERIAL | PRIMARY KEY | Unique chat log entry identifier |
| question | TEXT | NOT NULL | User's original question |
| answer | TEXT | NOT NULL | LLM-generated answer |
| sources | JSONB | NOT NULL | Array of source references: `[{title, section, url, chunk_text}, ...]` |
| error | TEXT | nullable | If request failed, error message for debugging |
| latency_ms | INTEGER | nullable | How long /ask took to respond (milliseconds) |
| created_at | TIMESTAMP | DEFAULT NOW() | When question was asked |

**Schema Example** (sources JSONB):
```json
[
  {
    "title": "Module 1: ROS 2 — Robotic Nervous System",
    "section": "chapters",
    "url": "/docs/chapters/ros2-nervous-system",
    "chunk_text": "ROS 2 is a middleware framework..."
  }
]
```

**Indexes**:
- `created_at DESC` (for querying recent chats)
- `question` (for pattern analysis)

**Validation Rules**:
- `question` must not be empty (>0 characters, <5000 characters)
- `answer` must not be empty
- `sources` must be valid JSON array

**Analytics Use Cases**:
- Track question patterns (e.g., "What questions do users ask most?")
- Measure latency trends
- Identify error patterns (null `answer` with non-null `error`)
- Monitor knowledge base gaps (questions without matching sources)

---

## Relationships

```
documents (1) ──── (M) chunks
    │
    └─── (url matches Docusaurus path)

chunks ──[embedding]──> Qdrant Collection (humanoid_docs)
    │
    └─── (referenced by chunk_text in chat_logs.sources)

chat_logs
    └─── (references chunks via sources array)
```

---

## Data Flow

### Ingestion Pipeline

1. **Scan** `/docs` folder recursively for `.md` files
2. **Parse** each file: extract frontmatter (title), content
3. **Chunk** content: split into 300-500 token segments using tiktoken
4. **Embed** each chunk: call OpenAI embedding API
5. **Store**:
   - PostgreSQL: Insert into `documents` + `chunks` tables
   - Qdrant: Upsert vectors with payload metadata
6. **Log**: Record ingestion completion, chunk count, duration

### Query Pipeline (RAG)

1. **User asks** question on chatbot UI
2. **Embed question** using OpenAI embedding API
3. **Search Qdrant**: Find top-K chunks (K=3-5) with highest cosine similarity
4. **Fetch metadata**: Look up chunk details in PostgreSQL (chunk_text, title, url)
5. **Generate**: Call OpenAI GPT-4 with question + context chunks
6. **Return**: Answer + sources
7. **Log**: Insert into `chat_logs` table

---

## Constraints & Notes

- **No user authentication (v1)**: ChatLogs not linked to user_id; all questions are anonymous
- **Time-series data**: ChatLog is append-only; used for analytics, not for user-facing history (frontend session state)
- **Document updates**: Ingestion is full-refresh (drop old chunks, insert new) to avoid stale vectors
- **Vector cleanup**: When document is deleted from filesystem, its chunks are deleted from both Qdrant and PostgreSQL
- **JSONB storage**: Sources stored as JSON in PostgreSQL for flexibility; no separate normalization table

