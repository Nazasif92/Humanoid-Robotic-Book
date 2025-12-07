"""Unit tests for app configuration and settings."""

import os
import pytest
from app.config import Settings


class TestSettings:
    """Test Settings configuration."""

    def test_settings_defaults(self):
        """Test default configuration values."""
        settings = Settings()
        assert settings.backend_host == "0.0.0.0"
        assert settings.backend_port == 8000
        assert settings.log_level == "INFO"
        assert settings.rag_top_k == 3
        assert settings.rag_min_similarity == 0.5

    def test_settings_from_env(self, monkeypatch):
        """Test settings loaded from environment variables."""
        monkeypatch.setenv("BACKEND_PORT", "9000")
        monkeypatch.setenv("LOG_LEVEL", "DEBUG")
        monkeypatch.setenv("RAG_TOP_K", "5")

        settings = Settings()
        assert settings.backend_port == 9000
        assert settings.log_level == "DEBUG"
        assert settings.rag_top_k == 5

    def test_settings_log_level_validation(self, monkeypatch):
        """Test log level validation."""
        monkeypatch.setenv("LOG_LEVEL", "INVALID")
        with pytest.raises(ValueError):
            Settings()

    def test_settings_similarity_threshold_validation(self, monkeypatch):
        """Test similarity threshold validation."""
        monkeypatch.setenv("RAG_MIN_SIMILARITY", "1.5")
        with pytest.raises(ValueError):
            Settings()

    def test_settings_rate_limit_period_validation(self, monkeypatch):
        """Test rate limit period validation."""
        monkeypatch.setenv("RATE_LIMIT_PERIOD", "0")
        with pytest.raises(ValueError):
            Settings()

    def test_settings_openai_timeout_validation(self, monkeypatch):
        """Test OpenAI timeout validation."""
        monkeypatch.setenv("OPENAI_TIMEOUT", "-5")
        with pytest.raises(ValueError):
            Settings()

    def test_settings_frontend_url_format(self, monkeypatch):
        """Test frontend URL must be valid."""
        monkeypatch.setenv("FRONTEND_URL", "not-a-url")
        with pytest.raises(ValueError):
            Settings()

    def test_settings_qdrant_url_required(self, monkeypatch):
        """Test that Qdrant URL is required."""
        monkeypatch.delenv("QDRANT_URL", raising=False)
        with pytest.raises(ValueError):
            Settings()

    def test_settings_neon_connection_string_required(self, monkeypatch):
        """Test that Neon connection string is required."""
        monkeypatch.delenv("NEON_CONNECTION_STRING", raising=False)
        with pytest.raises(ValueError):
            Settings()
