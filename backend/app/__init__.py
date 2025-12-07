"""RAG Chatbot Backend Application."""

__version__ = "1.0.0"
__author__ = "Humanoid Robotics Book Team"

from .config import Settings, settings
from .models import (
    AskRequest,
    AskResponse,
    Source,
    ErrorResponse,
    HealthResponse,
    Document,
    Chunk,
    ChatLog,
    IngestRequest,
    IngestResponse,
    ChatHistoryRequest,
    ChatHistoryResponse,
)
from .neon_client import NeonClient, neon_client
from .qdrant_client import QdrantClient, qdrant_client

__all__ = [
    "Settings",
    "settings",
    "AskRequest",
    "AskResponse",
    "Source",
    "ErrorResponse",
    "HealthResponse",
    "Document",
    "Chunk",
    "ChatLog",
    "IngestRequest",
    "IngestResponse",
    "ChatHistoryRequest",
    "ChatHistoryResponse",
    "NeonClient",
    "neon_client",
    "QdrantClient",
    "qdrant_client",
]
