"""Integration tests for FastAPI endpoints."""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch, MagicMock
from app.main import app
from app.models import AskRequest, AskResponse, Source


@pytest.fixture
def client():
    """Create FastAPI test client."""
    return TestClient(app)


class TestRootEndpoint:
    """Test GET / endpoint."""

    def test_root_endpoint(self, client):
        """Test root endpoint returns service info."""
        response = client.get("/")
        assert response.status_code == 200
        data = response.json()
        assert data["name"] == "RAG Chatbot Backend"
        assert data["version"] == "1.0.0"
        assert "endpoints" in data
        assert "/health" in data["endpoints"].values()


class TestHealthEndpoint:
    """Test GET /health endpoint."""

    @patch("app.main.neon_client.health_check")
    @patch("app.main.qdrant_client.health_check")
    @pytest.mark.asyncio
    async def test_health_check_all_healthy(self, mock_qdrant_health, mock_neon_health, client):
        """Test health check when all services are healthy."""
        mock_neon_health.return_value = True
        mock_qdrant_health.return_value = True

        response = client.get("/health")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "ok"
        assert "checks" in data
        assert "timestamp" in data

    @patch("app.main.neon_client.health_check")
    @patch("app.main.qdrant_client.health_check")
    @pytest.mark.asyncio
    async def test_health_check_degraded(self, mock_qdrant_health, mock_neon_health, client):
        """Test health check when one service is down."""
        mock_neon_health.return_value = False
        mock_qdrant_health.return_value = True

        response = client.get("/health")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "degraded"


class TestAskEndpoint:
    """Test POST /ask endpoint."""

    @patch("app.main.rag_pipeline.answer_question")
    @pytest.mark.asyncio
    async def test_ask_endpoint_success(self, mock_answer, client):
        """Test successful question answering."""
        # Mock the RAG pipeline
        sources = [
            Source(
                title="ROS 2 Basics",
                section="Introduction",
                url="/docs/ros2-basics"
            )
        ]
        mock_answer.return_value = (
            "ROS 2 is a middleware for robotics.",
            sources
        )

        request_data = {"question": "What is ROS 2?"}
        response = client.post("/ask", json=request_data)

        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "success"
        assert "answer" in data
        assert "sources" in data
        assert "latency_ms" in data

    def test_ask_endpoint_empty_question(self, client):
        """Test ask endpoint with empty question."""
        request_data = {"question": ""}
        response = client.post("/ask", json=request_data)
        assert response.status_code == 422  # Validation error

    def test_ask_endpoint_missing_question(self, client):
        """Test ask endpoint without question field."""
        response = client.post("/ask", json={})
        assert response.status_code == 422

    @patch("app.main.rag_pipeline.answer_question")
    @pytest.mark.asyncio
    async def test_ask_endpoint_with_selected_text(self, mock_answer, client):
        """Test ask endpoint with selected text context."""
        sources = []
        mock_answer.return_value = ("Answer based on context", sources)

        request_data = {
            "question": "Explain this",
            "selected_text": "ROS 2 is a middleware..."
        }
        response = client.post("/ask", json=request_data)

        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "success"
        # Verify selected_text was passed to RAG pipeline
        mock_answer.assert_called_once()
        call_args = mock_answer.call_args
        assert call_args.kwargs["selected_text"] == "ROS 2 is a middleware..."

    @patch("app.main.rag_pipeline.answer_question")
    @pytest.mark.asyncio
    async def test_ask_endpoint_pipeline_error(self, mock_answer, client):
        """Test ask endpoint when RAG pipeline fails."""
        mock_answer.side_effect = Exception("OpenAI API error")

        request_data = {"question": "What is ROS?"}
        response = client.post("/ask", json=request_data)

        # Should return 500 error
        assert response.status_code == 500
        data = response.json()
        assert data["status"] == "error"


class TestChatHistoryEndpoints:
    """Test chat history endpoints."""

    @patch("app.main.neon_client.get_chat_history")
    @pytest.mark.asyncio
    async def test_get_chat_history(self, mock_get_history, client):
        """Test GET /chat-history endpoint."""
        mock_get_history.return_value = (
            100,  # total count
            [
                {
                    "id": 1,
                    "question": "What is ROS?",
                    "answer": "ROS is...",
                    "latency_ms": 1200,
                    "created_at": "2024-01-15T10:30:00"
                }
            ]
        )

        response = client.get("/chat-history?limit=50&offset=0")
        assert response.status_code == 200
        data = response.json()
        assert data["total"] == 100
        assert data["limit"] == 50
        assert len(data["items"]) == 1

    @patch("app.main.neon_client.get_chat_history")
    @pytest.mark.asyncio
    async def test_get_chat_history_limit_cap(self, mock_get_history, client):
        """Test chat history limit is capped at 100."""
        mock_get_history.return_value = (0, [])

        # Request with limit > 100 should be capped
        response = client.get("/chat-history?limit=500&offset=0")
        assert response.status_code == 200
        # The endpoint caps limit to 100 before querying
        mock_get_history.assert_called_once()
        call_args = mock_get_history.call_args
        assert call_args.kwargs["limit"] <= 100

    @patch("app.main.neon_client.get_chat_by_id")
    @pytest.mark.asyncio
    async def test_get_chat_entry(self, mock_get_chat, client):
        """Test GET /chat-history/{chat_id} endpoint."""
        mock_get_chat.return_value = {
            "id": 1,
            "question": "What is ROS?",
            "answer": "ROS is...",
            "latency_ms": 1200,
            "created_at": "2024-01-15T10:30:00"
        }

        response = client.get("/chat-history/1")
        assert response.status_code == 200
        data = response.json()
        assert data["id"] == 1

    @patch("app.main.neon_client.get_chat_by_id")
    @pytest.mark.asyncio
    async def test_get_chat_entry_not_found(self, mock_get_chat, client):
        """Test GET chat entry returns 404 if not found."""
        mock_get_chat.return_value = None

        response = client.get("/chat-history/999")
        assert response.status_code == 404
