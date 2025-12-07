# Feature Specification: RAG Chatbot Integration

**Feature Branch**: `005-rag-chatbot-integration`
**Created**: 2025-12-07
**Status**: Draft
**Input**: Integrate RAG Chatbot with FastAPI backend, Qdrant + Neon, and Docusaurus frontend

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Documentation (Priority: P1)

A user visits the Humanoid-Robotics-Book website and wants to get answers to specific questions about robotics, ROS 2, or other topics covered in the documentation. Instead of manually searching through dozens of pages, they can ask the chatbot.

**Why this priority**: Core feature that delivers immediate value. Users can search and get context-specific answers without navigating documentation.

**Independent Test**: User types a question → chatbot retrieves relevant docs from knowledge base → returns answer with sources. This can be fully tested by submitting a question and validating the response includes both an answer and citations.

**Acceptance Scenarios**:

1. **Given** a user has navigated to the Chatbot page, **When** they type a question and click submit, **Then** the system displays a loading indicator and fetches an answer within 5 seconds
2. **Given** a question has been answered, **When** the response displays, **Then** it includes cited sources (document title, section, URL) from the knowledge base
3. **Given** a question is outside the knowledge base scope, **When** the chatbot responds, **Then** it clearly states "I don't have information about this" rather than generating false information

---

### User Story 2 - Highlight Text and Ask Context-Specific Questions (Priority: P2)

A user is reading a section of the documentation and wants to ask a follow-up question about that specific content. They can select text and pass it to the chatbot with additional context.

**Why this priority**: Enhances user experience by enabling precise context. Improves answer quality by providing relevant excerpts directly from the source.

**Independent Test**: User selects text on a doc page → opens chatbot with context → asks follow-up question. System uses selected text as context for RAG retrieval. Can be tested by validating that responses reference the selected text.

**Acceptance Scenarios**:

1. **Given** a user has selected text from documentation, **When** they choose "Ask about this" option, **Then** the selected text is passed to the chatbot context
2. **Given** selected text is provided, **When** the chatbot answers, **Then** the answer incorporates the selected text as additional context (visible in sources)
3. **Given** no text is selected, **When** the user asks a question, **Then** the system still functions normally without context (graceful fallback)

---

### User Story 3 - View Chat History and Previous Questions (Priority: P3)

A user wants to reference previous questions they asked or see a history of their interactions with the chatbot.

**Why this priority**: Nice-to-have for user engagement. Enables users to track their learning journey and revisit previous answers.

**Independent Test**: After multiple questions, user accesses chat history. Can be tested by verifying previous Q&A pairs are displayed and persist across sessions.

**Acceptance Scenarios**:

1. **Given** a user has asked multiple questions, **When** they access the chat history panel, **Then** all previous questions and answers are displayed in reverse chronological order
2. **Given** a previous chat entry is displayed, **When** the user clicks it, **Then** the context reloads and the full answer is visible

---

### Edge Cases

- What happens when the knowledge base has no relevant documents for a user's question?
- How does the system handle malformed or empty questions?
- What occurs if the backend service is unavailable?
- How does the chatbot handle very long questions (>5000 characters)?
- What happens when a document in the knowledge base is updated—does the chatbot immediately reflect changes?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a `/ask` endpoint that accepts a question and optional selected_text, returns an answer and sources from the knowledge base
- **FR-002**: System MUST embed all markdown documents from the `/docs` folder into vectors and store them in Qdrant with metadata
- **FR-003**: System MUST store document metadata (title, section, URL, chunk text) in a Neon PostgreSQL database
- **FR-004**: System MUST retrieve top-K matching chunks from Qdrant based on semantic similarity to the user's question
- **FR-005**: System MUST fetch full chunk text and metadata from Neon for retrieved vectors
- **FR-006**: System MUST call OpenAI GPT-4 API with the user's question and context to generate a natural language answer
- **FR-007**: System MUST return JSON responses with both an `answer` field and a `sources` array (containing document titles, sections, and URLs)
- **FR-008**: Backend MUST enable CORS for the Vercel frontend domain to allow cross-origin requests
- **FR-009**: System MUST provide a health check endpoint to validate backend connectivity
- **FR-010**: Frontend MUST display a chatbot interface with a text input for questions and an answer display area
- **FR-011**: Frontend MUST show cited sources below the chatbot answer with clickable links to the original documentation
- **FR-012**: Frontend MUST support optional selected text input for context-specific queries
- **FR-013**: Ingestion pipeline MUST load all `.md` files from the Docusaurus `/docs` folder recursively
- **FR-014**: Ingestion pipeline MUST split documents into chunks of 300–500 tokens to balance context and retrieval performance
- **FR-015**: Ingestion pipeline MUST embed each chunk using OpenAI's embedding model
- **FR-016**: Ingestion pipeline MUST store vectors in Qdrant with associated metadata
- **FR-017**: Ingestion pipeline MUST store chunk text and metadata in Neon for later retrieval

### Key Entities *(include if feature involves data)*

- **Document**: Represents a markdown file from the Docusaurus `/docs` folder. Attributes: `title` (extracted from frontmatter), `section` (folder path), `url` (generated based on Docusaurus routing), `content` (full markdown text)
- **Chunk**: A segment of a document, split for embedding and retrieval. Attributes: `doc_id` (reference to parent document), `chunk_text` (300–500 tokens), `embedding` (vector), `section` (document section), `order` (position in document)
- **ChatLog**: Records a question-answer pair for audit and analytics. Attributes: `question`, `answer`, `sources` (JSON array), `created_at` (timestamp), `user_id` (optional, for future multi-user support)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chatbot answers user questions within 5 seconds from submission to answer display
- **SC-002**: At least 85% of answers are sourced from the knowledge base with accurate citations
- **SC-003**: Users can navigate from a cited source to the original documentation page within 2 clicks
- **SC-004**: The chatbot interface is accessible from any page on the Humanoid-Robotics-Book website (e.g., via floating button or navbar link)
- **SC-005**: Ingestion pipeline successfully processes all markdown files in the `/docs` folder without errors
- **SC-006**: Backend service remains available with 99% uptime on the chosen platform (Railway)
- **SC-007**: Frontend renders the chatbot UI without layout shifts or blocking the main documentation content
- **SC-008**: Users receive clear feedback if the backend is unavailable (e.g., error message instead of a frozen interface)
- **SC-009**: System supports at least 10 concurrent chatbot requests without degradation in response time
- **SC-010**: Answers include at least one cited source for 95% of questions with matching knowledge base content

## Clarifications

### Session 2025-12-07

- Q: Knowledge base refresh strategy? → A: Scheduled automatic refresh every 24 hours
- Q: Observability & logging requirements? → A: Standard operational metrics (query success/failure rates, latency p50/p95/p99, API errors, ingestion job status, chat volume)
- Q: Failure handling & retry policies? → A: Exponential backoff with service-specific timeouts (OpenAI: 30s max, Qdrant/Neon: 5s max, up to 3 retries)

## Assumptions

- OpenAI API key will be provided and kept secure in environment variables
- Neon and Qdrant instances are created and accessible before ingestion begins
- The Docusaurus project structure follows standard conventions (docs are in `/docs` folder)
- Users are accessing the chatbot from the Vercel-deployed frontend
- Markdown documents contain front matter with `title` for metadata extraction
- The knowledge base is refreshed via automatic scheduled ingestion every 24 hours (cron job or cloud scheduler)

## Non-Functional Requirements

### Performance
- Question answering latency: p95 < 5 seconds
- Ingestion speed: 1000 chunks/minute
- Concurrent users: minimum 10 simultaneous requests

### Reliability
- Backend uptime: 99% monthly
- Graceful degradation: chatbot informs user if backend is unavailable
- No data loss: all chat logs persisted
- Service-specific timeout policies:
  - OpenAI API: 30 second timeout, exponential backoff retry (max 3 attempts)
  - Qdrant: 5 second timeout, exponential backoff retry (max 3 attempts)
  - Neon PostgreSQL: 5 second timeout, exponential backoff retry (max 3 attempts)
  - After all retries exhausted: return user-friendly error message

### Security
- API keys stored as environment variables, never in code
- CORS restricted to Vercel frontend domain
- Rate limiting on `/ask` endpoint (e.g., 20 requests/minute per IP)
- No sensitive user data stored in chat logs

### Observability
- Track query success/failure rates (target: >95% success rate)
- Monitor latency percentiles: p50 < 2s, p95 < 5s, p99 < 8s
- Log and alert on API errors (OpenAI, Qdrant, Neon timeouts)
- Monitor ingestion job status and duration (target: <1 hour for full run)
- Track chat volume and question patterns for insights

### Scalability
- Knowledge base can support up to 10,000 documents/chunks
- Vector search should complete within 1 second for typical queries
- Database connection pooling for PostgreSQL

## Out of Scope

- User authentication or multi-user support (for v1)
- Feedback mechanism (thumbs up/down on answers)
- Admin dashboard for knowledge base management
- Mobile app version
- Real-time collaboration or chat between users
