"""Unit tests for Pydantic models and validation."""

import pytest
from app.models import (
    AskRequest,
    AskResponse,
    Source,
    ErrorResponse,
    Document,
    DocumentCreate,
    Chunk,
    ChunkCreate,
)


class TestAskRequest:
    """Test AskRequest model validation."""

    def test_ask_request_valid(self):
        """Test valid ask request."""
        req = AskRequest(question="What is ROS 2?")
        assert req.question == "What is ROS 2?"
        assert req.selected_text is None

    def test_ask_request_with_selected_text(self):
        """Test ask request with selected text context."""
        req = AskRequest(
            question="Explain this",
            selected_text="ROS 2 is a flexible middleware..."
        )
        assert req.selected_text == "ROS 2 is a flexible middleware..."

    def test_ask_request_question_required(self):
        """Test question is required."""
        with pytest.raises(ValueError):
            AskRequest(question="")

    def test_ask_request_question_max_length(self):
        """Test question max length (5000 chars)."""
        long_question = "a" * 5001
        with pytest.raises(ValueError):
            AskRequest(question=long_question)

    def test_ask_request_selected_text_max_length(self):
        """Test selected text max length (2000 chars)."""
        long_text = "a" * 2001
        with pytest.raises(ValueError):
            AskRequest(question="What?", selected_text=long_text)


class TestAskResponse:
    """Test AskResponse model."""

    def test_ask_response_valid(self):
        """Test valid ask response."""
        sources = [
            Source(
                title="ROS 2 Basics",
                section="Introduction",
                url="/docs/ros2-basics",
            )
        ]
        resp = AskResponse(
            answer="ROS 2 is a middleware...",
            sources=sources,
            status="success",
            latency_ms=1250
        )
        assert resp.answer == "ROS 2 is a middleware..."
        assert resp.status == "success"
        assert resp.latency_ms == 1250
        assert len(resp.sources) == 1

    def test_ask_response_empty_sources(self):
        """Test response with no sources."""
        resp = AskResponse(
            answer="Answer",
            sources=[],
            status="success",
            latency_ms=500
        )
        assert len(resp.sources) == 0

    def test_ask_response_partial_status(self):
        """Test partial status for degraded response."""
        resp = AskResponse(
            answer="Limited answer",
            sources=[],
            status="partial",
            latency_ms=2000
        )
        assert resp.status == "partial"


class TestSource:
    """Test Source model."""

    def test_source_valid(self):
        """Test valid source."""
        src = Source(
            title="Chapter 1",
            section="Getting Started",
            url="/docs/chapter1",
        )
        assert src.title == "Chapter 1"
        assert src.section == "Getting Started"

    def test_source_with_chunk_text(self):
        """Test source with optional chunk text."""
        src = Source(
            title="Chapter 1",
            section="Getting Started",
            url="/docs/chapter1",
            chunk_text="ROS 2 fundamentals..."
        )
        assert src.chunk_text == "ROS 2 fundamentals..."


class TestDocumentCreate:
    """Test DocumentCreate validation."""

    def test_document_create_valid(self):
        """Test valid document creation."""
        doc = DocumentCreate(
            title="ROS 2 Guide",
            section="docs",
            url="/docs/ros2-guide",
            content="# ROS 2 Guide\n\n..."
        )
        assert doc.title == "ROS 2 Guide"

    def test_document_create_title_required(self):
        """Test title is required."""
        with pytest.raises(ValueError):
            DocumentCreate(
                title="",
                section="docs",
                url="/docs/test",
                content="content"
            )

    def test_document_create_url_format(self):
        """Test URL must be valid path."""
        with pytest.raises(ValueError):
            DocumentCreate(
                title="Test",
                section="docs",
                url="not-a-url",
                content="content"
            )


class TestChunkCreate:
    """Test ChunkCreate validation."""

    def test_chunk_create_valid(self):
        """Test valid chunk creation."""
        chunk = ChunkCreate(
            doc_id=1,
            chunk_text="This is a chunk of text with sufficient tokens for RAG.",
            chunk_index=0,
            section="introduction"
        )
        assert chunk.doc_id == 1
        assert chunk.chunk_index == 0

    def test_chunk_create_text_minimum_tokens(self):
        """Test chunk text has minimum tokens."""
        short_text = "Short"
        with pytest.raises(ValueError):
            ChunkCreate(
                doc_id=1,
                chunk_text=short_text,
                chunk_index=0,
                section="intro"
            )

    def test_chunk_create_text_maximum_tokens(self):
        """Test chunk text respects maximum tokens."""
        # Create text with ~1600 tokens (approximately 6400 chars)
        long_text = "word " * 1600
        with pytest.raises(ValueError):
            ChunkCreate(
                doc_id=1,
                chunk_text=long_text,
                chunk_index=0,
                section="intro"
            )
