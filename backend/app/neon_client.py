"""Neon PostgreSQL client for RAG Chatbot."""

import logging
from typing import List, Optional, Dict, Any
from datetime import datetime

import asyncpg
from asyncpg.pool import Pool

from .config import settings

logger = logging.getLogger(__name__)


class NeonClient:
    """Async PostgreSQL client for Neon."""

    def __init__(self) -> None:
        """Initialize Neon client."""
        self.pool: Optional[Pool] = None
        self.connection_string = settings.neon_connection_string

    async def connect(self) -> None:
        """Establish connection pool to Neon."""
        try:
            self.pool = await asyncpg.create_pool(
                self.connection_string,
                min_size=1,
                max_size=settings.max_db_connections,
                command_timeout=settings.neon_timeout,
            )
            logger.info("Connected to Neon PostgreSQL")
        except Exception as e:
            logger.error(f"Failed to connect to Neon: {e}")
            raise

    async def disconnect(self) -> None:
        """Close connection pool."""
        if self.pool:
            await self.pool.close()
            logger.info("Disconnected from Neon PostgreSQL")

    async def init_db(self) -> None:
        """Initialize database schema."""
        if not self.pool:
            raise RuntimeError("Not connected to database")

        async with self.pool.acquire() as conn:
            # Create documents table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS documents (
                    id SERIAL PRIMARY KEY,
                    title TEXT NOT NULL,
                    section TEXT NOT NULL,
                    url TEXT NOT NULL UNIQUE,
                    content TEXT NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """)
            logger.info("Created 'documents' table")

            # Create chunks table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS chunks (
                    id SERIAL PRIMARY KEY,
                    doc_id INTEGER NOT NULL REFERENCES documents(id) ON DELETE CASCADE,
                    chunk_text TEXT NOT NULL,
                    chunk_index INTEGER NOT NULL,
                    section TEXT NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    UNIQUE(doc_id, chunk_index)
                );
            """)
            logger.info("Created 'chunks' table")

            # Create chat_logs table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS chat_logs (
                    id SERIAL PRIMARY KEY,
                    question TEXT NOT NULL,
                    answer TEXT,
                    sources JSONB DEFAULT '[]'::jsonb,
                    error TEXT,
                    latency_ms INTEGER,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """)
            logger.info("Created 'chat_logs' table")

            # Create indexes for performance
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_chunks_doc_id ON chunks(doc_id);
            """)
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_chat_logs_created_at ON chat_logs(created_at DESC);
            """)
            logger.info("Created database indexes")

    async def create_document(self, document_data: Dict[str, Any]) -> int:
        """Create a document in PostgreSQL.

        Args:
            document_data: Document data (title, section, url, content)

        Returns:
            Document ID
        """
        if not self.pool:
            raise RuntimeError("Not connected to database")

        async with self.pool.acquire() as conn:
            doc_id = await conn.fetchval(
                """
                INSERT INTO documents (title, section, url, content)
                VALUES ($1, $2, $3, $4)
                ON CONFLICT (url) DO UPDATE SET
                    content = $4,
                    updated_at = CURRENT_TIMESTAMP
                RETURNING id;
                """,
                document_data["title"],
                document_data["section"],
                document_data["url"],
                document_data["content"],
            )
            logger.info(f"Created/updated document: {document_data['url']}")
            return doc_id

    async def create_chunk(self, chunk_data: Dict[str, Any]) -> int:
        """Create a chunk in PostgreSQL.

        Args:
            chunk_data: Chunk data (doc_id, chunk_text, chunk_index, section)

        Returns:
            Chunk ID
        """
        if not self.pool:
            raise RuntimeError("Not connected to database")

        async with self.pool.acquire() as conn:
            chunk_id = await conn.fetchval(
                """
                INSERT INTO chunks (doc_id, chunk_text, chunk_index, section)
                VALUES ($1, $2, $3, $4)
                RETURNING id;
                """,
                chunk_data["doc_id"],
                chunk_data["chunk_text"],
                chunk_data["chunk_index"],
                chunk_data["section"],
            )
            return chunk_id

    async def get_chunks_by_ids(self, chunk_ids: List[int]) -> List[Dict[str, Any]]:
        """Retrieve chunks by IDs.

        Args:
            chunk_ids: List of chunk IDs

        Returns:
            List of chunk dictionaries with metadata
        """
        if not self.pool:
            raise RuntimeError("Not connected to database")

        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT c.id, c.chunk_text, c.section, d.title, d.url
                FROM chunks c
                JOIN documents d ON c.doc_id = d.id
                WHERE c.id = ANY($1);
                """,
                chunk_ids,
            )
            return [dict(row) for row in rows]

    async def log_chat(self, chat_data: Dict[str, Any]) -> int:
        """Log a chat interaction.

        Args:
            chat_data: Chat data (question, answer, sources, error, latency_ms)

        Returns:
            Chat log ID
        """
        if not self.pool:
            raise RuntimeError("Not connected to database")

        async with self.pool.acquire() as conn:
            log_id = await conn.fetchval(
                """
                INSERT INTO chat_logs (question, answer, sources, error, latency_ms)
                VALUES ($1, $2, $3, $4, $5)
                RETURNING id;
                """,
                chat_data.get("question"),
                chat_data.get("answer"),
                chat_data.get("sources", []),
                chat_data.get("error"),
                chat_data.get("latency_ms"),
            )
            return log_id

    async def get_chat_history(
        self, limit: int = 50, offset: int = 0
    ) -> tuple[int, List[Dict[str, Any]]]:
        """Retrieve chat history.

        Args:
            limit: Number of results
            offset: Number to skip

        Returns:
            Tuple of (total_count, chat_logs)
        """
        if not self.pool:
            raise RuntimeError("Not connected to database")

        async with self.pool.acquire() as conn:
            # Get total count
            total = await conn.fetchval("SELECT COUNT(*) FROM chat_logs;")

            # Get paginated results
            rows = await conn.fetch(
                """
                SELECT id, question, answer, sources, error, latency_ms, created_at
                FROM chat_logs
                ORDER BY created_at DESC
                LIMIT $1 OFFSET $2;
                """,
                limit,
                offset,
            )
            return total, [dict(row) for row in rows]

    async def get_chat_by_id(self, chat_id: int) -> Optional[Dict[str, Any]]:
        """Retrieve a specific chat log by ID.

        Args:
            chat_id: Chat log ID

        Returns:
            Chat log dictionary or None
        """
        if not self.pool:
            raise RuntimeError("Not connected to database")

        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT id, question, answer, sources, error, latency_ms, created_at
                FROM chat_logs
                WHERE id = $1;
                """,
                chat_id,
            )
            return dict(row) if row else None

    async def delete_document_chunks(self, doc_id: int) -> int:
        """Delete all chunks for a document.

        Args:
            doc_id: Document ID

        Returns:
            Number of chunks deleted
        """
        if not self.pool:
            raise RuntimeError("Not connected to database")

        async with self.pool.acquire() as conn:
            count = await conn.fetchval(
                "DELETE FROM chunks WHERE doc_id = $1;", doc_id
            )
            return count

    async def health_check(self) -> bool:
        """Check if database is accessible.

        Returns:
            True if healthy, False otherwise
        """
        try:
            if not self.pool:
                return False
            async with self.pool.acquire() as conn:
                await conn.fetchval("SELECT 1;")
            return True
        except Exception as e:
            logger.error(f"Neon health check failed: {e}")
            return False


# Global client instance
neon_client = NeonClient()
