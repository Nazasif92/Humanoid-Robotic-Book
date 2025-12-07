"""Configuration management for RAG Chatbot Backend."""

import logging
from typing import Optional

from pydantic_settings import BaseSettings
from pydantic import Field, field_validator


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # OpenAI Configuration
    openai_api_key: str = Field(..., description="OpenAI API key")
    openai_model: str = Field(
        default="gpt-4-turbo-preview",
        description="OpenAI model name for LLM calls"
    )
    openai_embedding_model: str = Field(
        default="text-embedding-3-small",
        description="OpenAI model for embeddings"
    )

    # Qdrant Configuration
    qdrant_url: str = Field(
        default="http://localhost:6333",
        description="Qdrant API URL (local or cloud)"
    )
    qdrant_api_key: Optional[str] = Field(
        default=None,
        description="Qdrant API key (required for cloud instances)"
    )
    qdrant_collection_name: str = Field(
        default="humanoid_docs",
        description="Qdrant collection name"
    )

    # Neon PostgreSQL Configuration
    neon_connection_string: str = Field(
        ...,
        description="Neon PostgreSQL connection string"
    )

    # Backend Configuration
    backend_host: str = Field(default="0.0.0.0", description="Backend server host")
    backend_port: int = Field(default=8000, description="Backend server port")
    log_level: str = Field(
        default="INFO",
        description="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)"
    )

    # Frontend Configuration (for CORS)
    frontend_url: str = Field(
        default="http://localhost:3000",
        description="Frontend URL for CORS whitelist"
    )

    # Ingestion Configuration
    docs_path: str = Field(
        default="../docs",
        description="Path to Docusaurus /docs folder"
    )
    ingestion_batch_size: int = Field(
        default=100,
        description="Batch size for parallel embedding"
    )

    # Database Configuration
    max_db_connections: int = Field(
        default=20,
        description="Maximum PostgreSQL connection pool size"
    )

    # Rate Limiting
    rate_limit_requests: int = Field(
        default=20,
        description="Requests allowed per rate limit period"
    )
    rate_limit_period: int = Field(
        default=60,
        description="Rate limit period in seconds"
    )

    # Feature Flags
    enable_chat_history: bool = Field(
        default=True,
        description="Enable chat history endpoints"
    )
    enable_selected_text_context: bool = Field(
        default=True,
        description="Enable selected text context in RAG"
    )

    # RAG Configuration
    rag_top_k: int = Field(
        default=3,
        description="Number of chunks to retrieve from Qdrant"
    )
    rag_min_similarity: float = Field(
        default=0.5,
        description="Minimum similarity score for chunk retrieval"
    )

    # Timeout Configuration (seconds)
    openai_timeout: int = Field(
        default=30,
        description="OpenAI API timeout in seconds"
    )
    qdrant_timeout: int = Field(
        default=5,
        description="Qdrant API timeout in seconds"
    )
    neon_timeout: int = Field(
        default=5,
        description="Neon PostgreSQL timeout in seconds"
    )

    class Config:
        """Pydantic config."""
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False

    @field_validator("log_level")
    @classmethod
    def validate_log_level(cls, v: str) -> str:
        """Validate log level is valid."""
        valid_levels = {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"}
        if v.upper() not in valid_levels:
            raise ValueError(f"Invalid log level: {v}. Must be one of {valid_levels}")
        return v.upper()

    @field_validator("rag_min_similarity")
    @classmethod
    def validate_similarity_threshold(cls, v: float) -> float:
        """Validate similarity threshold is between 0 and 1."""
        if not 0 <= v <= 1:
            raise ValueError("rag_min_similarity must be between 0 and 1")
        return v


def get_settings() -> Settings:
    """Get application settings instance.

    Returns:
        Settings: Loaded configuration from environment

    Raises:
        ValueError: If required environment variables are missing
    """
    try:
        return Settings()
    except Exception as e:
        logging.error(f"Failed to load settings: {e}")
        raise


# Global settings instance
settings = get_settings()
