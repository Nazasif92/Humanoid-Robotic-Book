"""RAG Pipeline for semantic search and answer generation."""

import asyncio
import logging
import time
from typing import List, Dict, Any, Optional
from openai import AsyncOpenAI, RateLimitError, Timeout

from .config import settings
from .models import Source
from .neon_client import neon_client
from .qdrant_client import qdrant_client

logger = logging.getLogger(__name__)


class RAGPipeline:
    """Retrieval-Augmented Generation pipeline."""

    def __init__(self) -> None:
        """Initialize RAG pipeline."""
        self.openai_client = AsyncOpenAI(api_key=settings.openai_api_key)
        self.rag_top_k = settings.rag_top_k
        self.rag_min_similarity = settings.rag_min_similarity
        self.max_retries = 3
        self.base_delay = 1.0  # seconds

    async def embed_query(self, query: str) -> List[float]:
        """Embed user query using OpenAI.

        Args:
            query: User's question

        Returns:
            Embedding vector

        Raises:
            Exception: If embedding fails after retries
        """
        for attempt in range(self.max_retries):
            try:
                logger.info(f"Embedding query (attempt {attempt + 1}/{self.max_retries})")
                response = await asyncio.wait_for(
                    self.openai_client.embeddings.create(
                        input=query,
                        model=settings.openai_embedding_model,
                    ),
                    timeout=settings.openai_timeout,
                )
                embedding = response.data[0].embedding
                logger.info(f"Successfully embedded query ({len(embedding)} dims)")
                return embedding

            except Timeout:
                delay = self.base_delay * (2 ** attempt)
                if attempt < self.max_retries - 1:
                    logger.warning(
                        f"OpenAI timeout, retrying in {delay}s... (attempt {attempt + 1})"
                    )
                    await asyncio.sleep(delay)
                else:
                    logger.error("OpenAI embedding timeout after all retries")
                    raise

            except RateLimitError:
                delay = self.base_delay * (2 ** attempt)
                if attempt < self.max_retries - 1:
                    logger.warning(
                        f"OpenAI rate limited, retrying in {delay}s... (attempt {attempt + 1})"
                    )
                    await asyncio.sleep(delay)
                else:
                    logger.error("OpenAI rate limit after all retries")
                    raise

            except Exception as e:
                logger.error(f"Failed to embed query: {e}")
                raise

    async def search_knowledge_base(
        self, embedding: List[float]
    ) -> List[Dict[str, Any]]:
        """Search Qdrant for similar chunks.

        Args:
            embedding: Query embedding

        Returns:
            List of search results with id, score, payload
        """
        for attempt in range(self.max_retries):
            try:
                logger.info(f"Searching Qdrant (attempt {attempt + 1}/{self.max_retries})")
                results = qdrant_client.search(
                    embedding=embedding, top_k=self.rag_top_k
                )
                # Filter by minimum similarity
                filtered = [
                    r for r in results
                    if r["score"] >= self.rag_min_similarity
                ]
                logger.info(
                    f"Found {len(filtered)}/{len(results)} chunks above "
                    f"similarity threshold {self.rag_min_similarity}"
                )
                return filtered

            except Exception as e:
                delay = self.base_delay * (2 ** attempt)
                if attempt < self.max_retries - 1:
                    logger.warning(
                        f"Qdrant search failed, retrying in {delay}s: {e}"
                    )
                    await asyncio.sleep(delay)
                else:
                    logger.error(f"Qdrant search failed after all retries: {e}")
                    raise

    async def retrieve_chunks_from_db(
        self, chunk_ids: List[int]
    ) -> List[Dict[str, Any]]:
        """Fetch chunk details from PostgreSQL.

        Args:
            chunk_ids: List of chunk IDs

        Returns:
            List of chunk dictionaries with metadata
        """
        if not chunk_ids:
            return []

        for attempt in range(self.max_retries):
            try:
                logger.info(
                    f"Fetching {len(chunk_ids)} chunks from Neon "
                    f"(attempt {attempt + 1}/{self.max_retries})"
                )
                chunks = await neon_client.get_chunks_by_ids(chunk_ids)
                logger.info(f"Retrieved {len(chunks)} chunks from database")
                return chunks

            except Exception as e:
                delay = self.base_delay * (2 ** attempt)
                if attempt < self.max_retries - 1:
                    logger.warning(
                        f"Neon fetch failed, retrying in {delay}s: {e}"
                    )
                    await asyncio.sleep(delay)
                else:
                    logger.error(f"Neon fetch failed after all retries: {e}")
                    raise

    def _build_rag_prompt(
        self, question: str, chunks: List[Dict[str, Any]], selected_text: Optional[str]
    ) -> str:
        """Build system prompt with context chunks.

        Args:
            question: User's question
            chunks: Retrieved chunks
            selected_text: Optional selected text from user

        Returns:
            System prompt for LLM
        """
        context = "\n\n".join(
            [f"From {c['title']} (section: {c['section']}):\n{c['chunk_text']}" for c in chunks]
        )

        selected_context = ""
        if selected_text:
            selected_context = f"\n\nUser selected this text for additional context:\n{selected_text}"

        system_prompt = f"""You are a helpful assistant for the Humanoid Robotics Book.
Answer the user's question based on the provided documentation context.

If you don't know the answer based on the context, clearly say "I don't have information about this in the documentation."
Do NOT make up or hallucinate information not in the provided context.

Always cite the source documents when answering.

Documentation Context:
{context}
{selected_context}

User Question:
{question}

Provide a clear, concise answer with citations."""

        return system_prompt

    async def call_llm(
        self, question: str, chunks: List[Dict[str, Any]], selected_text: Optional[str]
    ) -> str:
        """Call OpenAI GPT-4 for answer generation.

        Args:
            question: User's question
            chunks: Retrieved context chunks
            selected_text: Optional user-selected text

        Returns:
            LLM-generated answer

        Raises:
            Exception: If LLM call fails after retries
        """
        system_prompt = self._build_rag_prompt(question, chunks, selected_text)

        for attempt in range(self.max_retries):
            try:
                logger.info(f"Calling OpenAI GPT-4 (attempt {attempt + 1}/{self.max_retries})")
                response = await asyncio.wait_for(
                    self.openai_client.chat.completions.create(
                        model=settings.openai_model,
                        messages=[
                            {"role": "system", "content": system_prompt},
                            {"role": "user", "content": question},
                        ],
                        temperature=0.7,
                        max_tokens=1000,
                    ),
                    timeout=settings.openai_timeout,
                )
                answer = response.choices[0].message.content
                logger.info("Successfully generated answer from LLM")
                return answer

            except Timeout:
                delay = self.base_delay * (2 ** attempt)
                if attempt < self.max_retries - 1:
                    logger.warning(
                        f"OpenAI timeout, retrying in {delay}s... (attempt {attempt + 1})"
                    )
                    await asyncio.sleep(delay)
                else:
                    logger.error("OpenAI LLM timeout after all retries")
                    raise

            except Exception as e:
                logger.error(f"Failed to call LLM: {e}")
                raise

    def _extract_sources(self, chunks: List[Dict[str, Any]]) -> List[Source]:
        """Extract source citations from chunks.

        Args:
            chunks: Retrieved chunks with metadata

        Returns:
            List of Source objects
        """
        sources = []
        seen = set()

        for chunk in chunks:
            # Avoid duplicate sources
            key = (chunk["title"], chunk["url"])
            if key not in seen:
                sources.append(
                    Source(
                        title=chunk["title"],
                        section=chunk["section"],
                        url=chunk["url"],
                        chunk_text=chunk.get("chunk_text", "")[:200],  # First 200 chars
                    )
                )
                seen.add(key)

        return sources

    async def answer_question(
        self, question: str, selected_text: Optional[str] = None
    ) -> tuple[str, List[Source]]:
        """End-to-end RAG pipeline: question -> answer + sources.

        Args:
            question: User's question
            selected_text: Optional selected context

        Returns:
            Tuple of (answer, sources)

        Raises:
            Exception: If any step fails
        """
        logger.info(f"Starting RAG pipeline for question: {question[:50]}...")

        # Step 1: Embed question
        embedding = await self.embed_query(question)

        # Step 2: Search Qdrant
        search_results = await self.search_knowledge_base(embedding)
        chunk_ids = [r["id"] for r in search_results]

        if not chunk_ids:
            logger.warning("No relevant chunks found in knowledge base")
            return (
                "I don't have information about this in the documentation.",
                [],
            )

        # Step 3: Retrieve chunk details from Neon
        chunks = await self.retrieve_chunks_from_db(chunk_ids)

        # Step 4: Generate answer with LLM
        answer = await self.call_llm(question, chunks, selected_text)

        # Step 5: Extract sources
        sources = self._extract_sources(chunks)

        logger.info(f"RAG pipeline complete. Answer length: {len(answer)} chars, {len(sources)} sources")
        return answer, sources
