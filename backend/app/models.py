"""Pydantic models for RAG Chatbot API."""

from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel, Field, field_validator


# ============================================================================
# Request/Response Models
# ============================================================================

class AskRequest(BaseModel):
    """Request model for POST /ask endpoint."""

    question: str = Field(
        ...,
        min_length=1,
        max_length=5000,
        description="User's question about documentation"
    )
    selected_text: Optional[str] = Field(
        default=None,
        max_length=2000,
        description="Optional context from selected text"
    )

    @field_validator("question")
    @classmethod
    def validate_question(cls, v: str) -> str:
        """Validate question is not empty or whitespace only."""
        if not v or not v.strip():
            raise ValueError("Question cannot be empty or whitespace only")
        return v.strip()

    @field_validator("selected_text")
    @classmethod
    def validate_selected_text(cls, v: Optional[str]) -> Optional[str]:
        """Validate selected text if provided."""
        if v is not None:
            if not v or not v.strip():
                return None
            return v.strip()
        return v


class Source(BaseModel):
    """Source citation for answer."""

    title: str = Field(..., description="Document title")
    section: str = Field(..., description="Document section/folder")
    url: str = Field(..., description="Docusaurus URL")
    chunk_text: Optional[str] = Field(
        default=None,
        description="Optional excerpt from source chunk"
    )


class AskResponse(BaseModel):
    """Response model for POST /ask endpoint."""

    answer: str = Field(..., description="LLM-generated answer")
    sources: List[Source] = Field(
        default_factory=list,
        description="List of source references"
    )
    status: str = Field(
        ...,
        pattern="^(success|partial|error)$",
        description="Response status"
    )
    latency_ms: Optional[int] = Field(
        default=None,
        ge=0,
        description="Request latency in milliseconds"
    )


class ErrorResponse(BaseModel):
    """Error response model."""

    status: str = Field(default="error", description="Always 'error'")
    error: str = Field(..., description="Error message")
    code: Optional[str] = Field(
        default=None,
        description="Error code for debugging"
    )


class HealthResponse(BaseModel):
    """Health check response model."""

    status: str = Field(
        ...,
        pattern="^(ok|degraded|error)$",
        description="Overall system health"
    )
    timestamp: datetime = Field(..., description="Health check timestamp")
    checks: dict = Field(
        default_factory=dict,
        description="Individual service checks (qdrant, neon, openai)"
    )


# ============================================================================
# Database Models (PostgreSQL)
# ============================================================================

class Document(BaseModel):
    """Document entity from PostgreSQL."""

    id: int = Field(..., description="Document ID")
    title: str = Field(..., description="Document title from frontmatter")
    section: str = Field(..., description="Folder path (e.g., 'chapters')")
    url: str = Field(..., description="Docusaurus URL")
    content: str = Field(..., description="Full markdown content")
    created_at: datetime = Field(..., description="Creation timestamp")
    updated_at: Optional[datetime] = Field(default=None, description="Update timestamp")

    class Config:
        """Pydantic config."""
        from_attributes = True


class DocumentCreate(BaseModel):
    """Create document request."""

    title: str = Field(..., description="Document title")
    section: str = Field(..., description="Document section")
    url: str = Field(..., description="Document URL")
    content: str = Field(..., description="Document content")

    @field_validator("title", "section", "url")
    @classmethod
    def validate_not_empty(cls, v: str) -> str:
        """Ensure fields are not empty."""
        if not v or not v.strip():
            raise ValueError("Field cannot be empty")
        return v.strip()


class Chunk(BaseModel):
    """Chunk entity from PostgreSQL."""

    id: int = Field(..., description="Chunk ID")
    doc_id: int = Field(..., description="Parent document ID")
    chunk_text: str = Field(..., description="Chunk content (300-500 tokens)")
    chunk_index: int = Field(..., ge=0, description="Position in document")
    section: str = Field(..., description="Document section")
    created_at: datetime = Field(..., description="Creation timestamp")

    class Config:
        """Pydantic config."""
        from_attributes = True


class ChunkCreate(BaseModel):
    """Create chunk request."""

    doc_id: int = Field(..., ge=1, description="Parent document ID")
    chunk_text: str = Field(..., description="Chunk content")
    chunk_index: int = Field(..., ge=0, description="Position in document")
    section: str = Field(..., description="Document section")

    @field_validator("chunk_text")
    @classmethod
    def validate_chunk_text(cls, v: str) -> str:
        """Validate chunk text length."""
        if not v or not v.strip():
            raise ValueError("Chunk text cannot be empty")
        # Rough validation: 50-1500 tokens (estimate: 1 token ≈ 4 characters)
        if len(v) < 200:  # ~50 tokens
            raise ValueError("Chunk too small (minimum ~50 tokens)")
        if len(v) > 6000:  # ~1500 tokens
            raise ValueError("Chunk too large (maximum ~1500 tokens)")
        return v.strip()


class ChatLog(BaseModel):
    """Chat log entry from PostgreSQL."""

    id: int = Field(..., description="Chat log ID")
    question: str = Field(..., description="User's question")
    answer: str = Field(..., description="LLM's answer")
    sources: List[dict] = Field(default_factory=list, description="Source references")
    error: Optional[str] = Field(default=None, description="Error message if failed")
    latency_ms: Optional[int] = Field(default=None, ge=0, description="Response latency")
    created_at: datetime = Field(..., description="Timestamp")

    class Config:
        """Pydantic config."""
        from_attributes = True


class ChatLogCreate(BaseModel):
    """Create chat log entry."""

    question: str = Field(..., description="User question")
    answer: Optional[str] = Field(default=None, description="Answer text")
    sources: List[dict] = Field(default_factory=list, description="Sources")
    error: Optional[str] = Field(default=None, description="Error message")
    latency_ms: Optional[int] = Field(default=None, ge=0, description="Latency")

    @field_validator("question")
    @classmethod
    def validate_question(cls, v: str) -> str:
        """Validate question."""
        if not v or not v.strip():
            raise ValueError("Question cannot be empty")
        return v.strip()


# ============================================================================
# Ingestion Models
# ============================================================================

class IngestRequest(BaseModel):
    """Request model for POST /ingest endpoint."""

    docs_path: Optional[str] = Field(
        default=None,
        description="Path to docs folder (uses DOCS_PATH env if not provided)"
    )
    batch_size: Optional[int] = Field(
        default=100,
        ge=10,
        le=500,
        description="Batch size for parallel embedding"
    )


class IngestResponse(BaseModel):
    """Response model for POST /ingest endpoint."""

    status: str = Field(
        ...,
        pattern="^(success|partial|error)$",
        description="Ingestion status"
    )
    chunks_created: int = Field(..., ge=0, description="Total chunks created")
    documents_processed: int = Field(..., ge=0, description="Documents processed")
    duration_seconds: float = Field(...., ge=0, description="Total duration")
    errors: Optional[List[str]] = Field(
        default=None,
        description="List of errors encountered"
    )


# ============================================================================
# Chat History Models
# ============================================================================

class ChatHistoryRequest(BaseModel):
    """Request for chat history pagination."""

    limit: int = Field(
        default=50,
        ge=1,
        le=100,
        description="Number of results to return"
    )
    offset: int = Field(
        default=0,
        ge=0,
        description="Number of results to skip"
    )


class ChatHistoryResponse(BaseModel):
    """Response with paginated chat history."""

    total: int = Field(..., description="Total number of chat logs")
    limit: int = Field(..., description="Limit requested")
    offset: int = Field(..., description="Offset requested")
    items: List[ChatLog] = Field(..., description="Chat log items")
