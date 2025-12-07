"""Pytest configuration and fixtures for backend tests."""

import asyncio
import os
from typing import Generator

import pytest
from unittest.mock import AsyncMock, MagicMock

# Configure test environment
os.environ["OPENAI_API_KEY"] = "test-key"
os.environ["NEON_CONNECTION_STRING"] = "postgresql://test:test@localhost/test"
os.environ["QDRANT_URL"] = "http://localhost:6333"


@pytest.fixture(scope="session")
def event_loop() -> Generator:
    """Create event loop for async tests."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def mock_openai_client() -> AsyncMock:
    """Fixture for mocked OpenAI client."""
    mock = AsyncMock()
    mock.create_embedding = AsyncMock(
        return_value=MagicMock(data=[MagicMock(embedding=[0.1] * 1536)])
    )
    mock.chat.completions.create = AsyncMock(
        return_value=MagicMock(
            choices=[MagicMock(message=MagicMock(content="Test answer"))]
        )
    )
    return mock


@pytest.fixture
def mock_qdrant_client() -> AsyncMock:
    """Fixture for mocked Qdrant client."""
    mock = AsyncMock()
    mock.search = AsyncMock(
        return_value=[
            MagicMock(
                id=1,
                score=0.95,
                payload={"doc_id": 1, "chunk_index": 0}
            )
        ]
    )
    mock.upsert = AsyncMock(return_value=MagicMock(operation_id=1))
    return mock


@pytest.fixture
def mock_neon_client() -> AsyncMock:
    """Fixture for mocked Neon PostgreSQL client."""
    mock = AsyncMock()
    mock.execute = AsyncMock(return_value=[])
    mock.fetch = AsyncMock(
        return_value=[
            {
                "id": 1,
                "doc_id": 1,
                "chunk_text": "Test chunk text",
                "title": "Test Document",
                "section": "chapters",
                "url": "/docs/test"
            }
        ]
    )
    return mock


@pytest.fixture
def sample_document() -> dict:
    """Fixture for sample document data."""
    return {
        "id": 1,
        "title": "Test Document",
        "section": "chapters",
        "url": "/docs/test",
        "content": "This is test content for a document."
    }


@pytest.fixture
def sample_chunk() -> dict:
    """Fixture for sample chunk data."""
    return {
        "id": 1,
        "doc_id": 1,
        "chunk_text": "This is a test chunk of 50 tokens.",
        "chunk_index": 0,
        "section": "chapters",
        "embedding": [0.1] * 1536
    }


@pytest.fixture
def sample_question() -> str:
    """Fixture for sample question."""
    return "What is ROS 2?"


@pytest.fixture
def sample_response() -> dict:
    """Fixture for sample API response."""
    return {
        "answer": "ROS 2 is a Robot Operating System...",
        "sources": [
            {
                "title": "Test Document",
                "section": "chapters",
                "url": "/docs/test",
                "chunk_text": "Test chunk content"
            }
        ],
        "status": "success",
        "latency_ms": 1500
    }
