"""Simple in-memory caching layer for RAG Chatbot.

Implements LRU (Least Recently Used) cache for:
- Embedding vectors (same question → same embedding)
- Search results (same embedding → same results)
- LLM responses (similar questions → cached answers)

Configuration via environment:
- CACHE_ENABLED: Enable/disable caching
- CACHE_MAX_SIZE: Maximum entries (default 1000)
- CACHE_TTL_SECONDS: Time to live (default 3600 = 1 hour)
"""

import hashlib
import logging
import time
from typing import Any, Optional, Dict, List
from functools import lru_cache
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class CacheEntry:
    """Single cache entry with TTL."""
    value: Any
    created_at: float

    def is_expired(self, ttl_seconds: int) -> bool:
        """Check if entry has expired."""
        return time.time() - self.created_at > ttl_seconds


class SimpleLRUCache:
    """Thread-safe LRU cache with TTL."""

    def __init__(self, max_size: int = 1000, ttl_seconds: int = 3600):
        """Initialize cache.

        Args:
            max_size: Maximum number of entries
            ttl_seconds: Time to live for each entry
        """
        self.max_size = max_size
        self.ttl_seconds = ttl_seconds
        self.cache: Dict[str, CacheEntry] = {}
        self.access_order: List[str] = []
        self.stats = {
            "hits": 0,
            "misses": 0,
            "evictions": 0,
        }

    def set(self, key: str, value: Any) -> None:
        """Set cache entry.

        Args:
            key: Cache key
            value: Value to cache
        """
        # Remove if already exists to update order
        if key in self.cache:
            self.access_order.remove(key)

        # Add new entry
        self.cache[key] = CacheEntry(value=value, created_at=time.time())
        self.access_order.append(key)

        # Evict oldest if over capacity
        while len(self.cache) > self.max_size:
            oldest_key = self.access_order.pop(0)
            del self.cache[oldest_key]
            self.stats["evictions"] += 1

        logger.debug(f"Cache SET {key} (size: {len(self.cache)}/{self.max_size})")

    def get(self, key: str) -> Optional[Any]:
        """Get cache entry.

        Args:
            key: Cache key

        Returns:
            Cached value or None if not found/expired
        """
        if key not in self.cache:
            self.stats["misses"] += 1
            logger.debug(f"Cache MISS {key}")
            return None

        entry = self.cache[key]

        # Check expiration
        if entry.is_expired(self.ttl_seconds):
            del self.cache[key]
            self.access_order.remove(key)
            self.stats["misses"] += 1
            logger.debug(f"Cache EXPIRED {key}")
            return None

        # Update access order (move to end)
        self.access_order.remove(key)
        self.access_order.append(key)

        self.stats["hits"] += 1
        logger.debug(f"Cache HIT {key}")
        return entry.value

    def clear(self) -> None:
        """Clear all cache entries."""
        self.cache.clear()
        self.access_order.clear()
        logger.info("Cache cleared")

    def get_stats(self) -> Dict[str, Any]:
        """Get cache statistics."""
        total = self.stats["hits"] + self.stats["misses"]
        hit_rate = (
            (self.stats["hits"] / total * 100) if total > 0 else 0
        )
        return {
            "size": len(self.cache),
            "max_size": self.max_size,
            "hits": self.stats["hits"],
            "misses": self.stats["misses"],
            "hit_rate_percent": hit_rate,
            "evictions": self.stats["evictions"],
            "ttl_seconds": self.ttl_seconds,
        }


# Global cache instances
embedding_cache = SimpleLRUCache(max_size=500, ttl_seconds=3600)
search_cache = SimpleLRUCache(max_size=1000, ttl_seconds=3600)
answer_cache = SimpleLRUCache(max_size=200, ttl_seconds=7200)


def hash_list(data: list) -> str:
    """Hash a list for use as cache key.

    Args:
        data: List to hash

    Returns:
        Hex digest
    """
    content = str(data).encode()
    return hashlib.sha256(content).hexdigest()[:16]


def cache_embedding(question: str, embedding: List[float]) -> None:
    """Cache embedding vector.

    Args:
        question: User question
        embedding: Embedding vector
    """
    key = hashlib.sha256(question.encode()).hexdigest()[:16]
    embedding_cache.set(key, embedding)


def get_cached_embedding(question: str) -> Optional[List[float]]:
    """Get cached embedding.

    Args:
        question: User question

    Returns:
        Embedding vector or None
    """
    key = hashlib.sha256(question.encode()).hexdigest()[:16]
    return embedding_cache.get(key)


def cache_search_results(
    embedding: List[float],
    results: List[Dict[str, Any]],
) -> None:
    """Cache search results.

    Args:
        embedding: Query embedding
        results: Search results
    """
    key = hash_list(embedding)
    search_cache.set(key, results)


def get_cached_search_results(
    embedding: List[float],
) -> Optional[List[Dict[str, Any]]]:
    """Get cached search results.

    Args:
        embedding: Query embedding

    Returns:
        Search results or None
    """
    key = hash_list(embedding)
    return search_cache.get(key)


def cache_answer(
    question: str,
    selected_text: Optional[str],
    answer: str,
) -> None:
    """Cache generated answer.

    Args:
        question: User question
        selected_text: Optional selected text
        answer: Generated answer
    """
    # Include selected_text in key for context-specific caching
    cache_key = f"{question}|{selected_text or ''}"
    key = hashlib.sha256(cache_key.encode()).hexdigest()[:16]
    answer_cache.set(key, answer)


def get_cached_answer(
    question: str,
    selected_text: Optional[str],
) -> Optional[str]:
    """Get cached answer.

    Args:
        question: User question
        selected_text: Optional selected text

    Returns:
        Cached answer or None
    """
    cache_key = f"{question}|{selected_text or ''}"
    key = hashlib.sha256(cache_key.encode()).hexdigest()[:16]
    return answer_cache.get(key)


def get_all_cache_stats() -> Dict[str, Any]:
    """Get statistics for all caches.

    Returns:
        Dictionary with stats for each cache
    """
    return {
        "embedding_cache": embedding_cache.get_stats(),
        "search_cache": search_cache.get_stats(),
        "answer_cache": answer_cache.get_stats(),
    }


def clear_all_caches() -> None:
    """Clear all caches."""
    embedding_cache.clear()
    search_cache.clear()
    answer_cache.clear()
    logger.info("All caches cleared")
