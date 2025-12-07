"""Qdrant vector database client for RAG Chatbot."""

import logging
from typing import List, Dict, Any, Optional

from qdrant_client import QdrantClient as QdrantAsyncClient
from qdrant_client.http.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
)

from .config import settings

logger = logging.getLogger(__name__)


class QdrantClient:
    """Client for Qdrant vector database."""

    def __init__(self) -> None:
        """Initialize Qdrant client."""
        self.client: Optional[QdrantAsyncClient] = None
        self.url = settings.qdrant_url
        self.api_key = settings.qdrant_api_key
        self.collection_name = settings.qdrant_collection_name

    def connect(self) -> None:
        """Connect to Qdrant."""
        try:
            self.client = QdrantAsyncClient(
                url=self.url,
                api_key=self.api_key,
                timeout=settings.qdrant_timeout,
            )
            logger.info(f"Connected to Qdrant at {self.url}")
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            raise

    def init_collection(self) -> None:
        """Initialize vector collection if not exists."""
        if not self.client:
            raise RuntimeError("Not connected to Qdrant")

        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with vector size 1536 (OpenAI embedding dimension)
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=1536,  # text-embedding-3-small output dimension
                        distance=Distance.COSINE,
                    ),
                )
                logger.info(
                    f"Created Qdrant collection: {self.collection_name}"
                )
            else:
                logger.info(
                    f"Collection '{self.collection_name}' already exists"
                )
        except Exception as e:
            logger.error(f"Failed to initialize collection: {e}")
            raise

    def upsert_vectors(
        self, vectors: List[Dict[str, Any]]
    ) -> None:
        """Upsert vectors into Qdrant.

        Args:
            vectors: List of vector dicts with 'id', 'embedding', 'payload'
        """
        if not self.client:
            raise RuntimeError("Not connected to Qdrant")

        try:
            points = [
                PointStruct(
                    id=v["id"],
                    vector=v["embedding"],
                    payload=v.get("payload", {}),
                )
                for v in vectors
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )
            logger.info(f"Upserted {len(points)} vectors to Qdrant")
        except Exception as e:
            logger.error(f"Failed to upsert vectors: {e}")
            raise

    def search(
        self, embedding: List[float], top_k: int = 3
    ) -> List[Dict[str, Any]]:
        """Search for similar vectors.

        Args:
            embedding: Query embedding vector
            top_k: Number of results to return

        Returns:
            List of search results with id, score, payload
        """
        if not self.client:
            raise RuntimeError("Not connected to Qdrant")

        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=embedding,
                limit=top_k,
                query_filter=None,
            )

            return [
                {
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload,
                }
                for result in results
            ]
        except Exception as e:
            logger.error(f"Search failed: {e}")
            raise

    def delete_by_payload(self, field: str, value: Any) -> None:
        """Delete vectors by payload filter.

        Args:
            field: Payload field name
            value: Field value to match
        """
        if not self.client:
            raise RuntimeError("Not connected to Qdrant")

        try:
            # Delete chunks with matching doc_id
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=Filter(
                    must=[
                        FieldCondition(
                            key=field,
                            match=MatchValue(value=value),
                        )
                    ]
                ),
            )
            logger.info(f"Deleted vectors with {field}={value}")
        except Exception as e:
            logger.error(f"Delete operation failed: {e}")
            raise

    def collection_info(self) -> Dict[str, Any]:
        """Get collection info.

        Returns:
            Collection metadata
        """
        if not self.client:
            raise RuntimeError("Not connected to Qdrant")

        try:
            info = self.client.get_collection(self.collection_name)
            return {
                "name": info.name,
                "points_count": info.points_count,
                "vectors_count": getattr(info, "vectors_count", None),
                "status": str(info.status),
            }
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            raise

    async def health_check(self) -> bool:
        """Check if Qdrant is accessible.

        Returns:
            True if healthy, False otherwise
        """
        try:
            if not self.client:
                return False
            # Try to get collection info
            self.client.get_collection(self.collection_name)
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return False

    def count_vectors(self) -> int:
        """Get number of vectors in collection.

        Returns:
            Vector count
        """
        if not self.client:
            raise RuntimeError("Not connected to Qdrant")

        try:
            info = self.client.get_collection(self.collection_name)
            return info.points_count
        except Exception as e:
            logger.error(f"Failed to count vectors: {e}")
            raise


# Global client instance
qdrant_client = QdrantClient()
