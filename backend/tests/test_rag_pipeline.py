"""Unit tests for RAG pipeline."""

import pytest
from unittest.mock import AsyncMock, patch, MagicMock
from app.rag_pipeline import RAGPipeline


@pytest.fixture
def rag_pipeline():
    """Create RAG pipeline instance."""
    return RAGPipeline()


class TestEmbedQuery:
    """Test query embedding."""

    @pytest.mark.asyncio
    async def test_embed_query_success(self, rag_pipeline):
        """Test successful query embedding."""
        with patch("app.rag_pipeline.rag_pipeline.openai_client.embeddings.create") as mock_embed:
            # Mock response
            mock_response = MagicMock()
            mock_response.data = [MagicMock(embedding=[0.1, 0.2, 0.3] * 512)]
            mock_embed.return_value = mock_response

            embedding = await rag_pipeline.embed_query("What is ROS 2?")
            assert len(embedding) > 0
            assert isinstance(embedding, list)

    @pytest.mark.asyncio
    async def test_embed_query_retry_on_timeout(self, rag_pipeline):
        """Test embedding retries on timeout."""
        with patch("app.rag_pipeline.rag_pipeline.openai_client.embeddings.create") as mock_embed:
            from openai import Timeout
            # First call times out, second succeeds
            mock_response = MagicMock()
            mock_response.data = [MagicMock(embedding=[0.1] * 1536)]
            mock_embed.side_effect = [Timeout(), mock_response]

            with patch("asyncio.sleep"):
                embedding = await rag_pipeline.embed_query("What is ROS 2?")
                assert len(embedding) > 0

    @pytest.mark.asyncio
    async def test_embed_query_max_retries_exceeded(self, rag_pipeline):
        """Test embedding fails after max retries."""
        from openai import Timeout
        with patch("app.rag_pipeline.rag_pipeline.openai_client.embeddings.create") as mock_embed:
            mock_embed.side_effect = Timeout()

            with patch("asyncio.sleep"):
                with pytest.raises(Timeout):
                    await rag_pipeline.embed_query("What is ROS 2?")


class TestSearchKnowledgeBase:
    """Test Qdrant search."""

    @pytest.mark.asyncio
    async def test_search_knowledge_base_success(self, rag_pipeline):
        """Test successful knowledge base search."""
        embedding = [0.1] * 1536
        mock_results = [
            {
                "id": 1,
                "score": 0.85,
                "payload": {"doc_id": 1, "chunk_index": 0}
            },
            {
                "id": 2,
                "score": 0.75,
                "payload": {"doc_id": 2, "chunk_index": 1}
            }
        ]

        with patch("app.rag_pipeline.qdrant_client.search") as mock_search:
            mock_search.return_value = mock_results
            results = await rag_pipeline.search_knowledge_base(embedding)
            assert len(results) == 2
            assert results[0]["score"] == 0.85

    @pytest.mark.asyncio
    async def test_search_filters_by_similarity(self, rag_pipeline):
        """Test search filters results by minimum similarity."""
        embedding = [0.1] * 1536
        mock_results = [
            {"id": 1, "score": 0.85, "payload": {}},
            {"id": 2, "score": 0.3, "payload": {}},  # Below threshold
            {"id": 3, "score": 0.75, "payload": {}}
        ]

        with patch("app.rag_pipeline.qdrant_client.search") as mock_search:
            mock_search.return_value = mock_results
            results = await rag_pipeline.search_knowledge_base(embedding)
            # Should only return scores >= min_similarity (0.5)
            assert len(results) == 2
            assert all(r["score"] >= rag_pipeline.rag_min_similarity for r in results)

    @pytest.mark.asyncio
    async def test_search_no_results(self, rag_pipeline):
        """Test search with no results."""
        embedding = [0.1] * 1536
        with patch("app.rag_pipeline.qdrant_client.search") as mock_search:
            mock_search.return_value = []
            results = await rag_pipeline.search_knowledge_base(embedding)
            assert len(results) == 0


class TestRetrieveChunksFromDb:
    """Test chunk retrieval from Neon."""

    @pytest.mark.asyncio
    async def test_retrieve_chunks_success(self, rag_pipeline):
        """Test successful chunk retrieval."""
        chunk_ids = [1, 2]
        mock_chunks = [
            {
                "id": 1,
                "chunk_text": "ROS 2 is a middleware",
                "title": "ROS Basics",
                "section": "intro",
                "url": "/docs/ros-basics"
            },
            {
                "id": 2,
                "chunk_text": "ROS 2 has multiple nodes",
                "title": "ROS Basics",
                "section": "architecture",
                "url": "/docs/ros-basics"
            }
        ]

        with patch("app.rag_pipeline.neon_client.get_chunks_by_ids") as mock_get:
            mock_get.return_value = mock_chunks
            chunks = await rag_pipeline.retrieve_chunks_from_db(chunk_ids)
            assert len(chunks) == 2
            assert chunks[0]["title"] == "ROS Basics"

    @pytest.mark.asyncio
    async def test_retrieve_chunks_empty(self, rag_pipeline):
        """Test retrieval with empty chunk IDs."""
        chunks = await rag_pipeline.retrieve_chunks_from_db([])
        assert len(chunks) == 0


class TestBuildRagPrompt:
    """Test RAG prompt construction."""

    def test_build_rag_prompt_basic(self, rag_pipeline):
        """Test basic RAG prompt building."""
        chunks = [
            {
                "title": "Chapter 1",
                "section": "Intro",
                "chunk_text": "ROS 2 is..."
            }
        ]
        prompt = rag_pipeline._build_rag_prompt(
            "What is ROS 2?",
            chunks,
            None
        )
        assert "ROS 2 is..." in prompt
        assert "What is ROS 2?" in prompt
        assert "From Chapter 1" in prompt

    def test_build_rag_prompt_with_selected_text(self, rag_pipeline):
        """Test RAG prompt with selected text context."""
        chunks = [
            {
                "title": "Chapter 1",
                "section": "Intro",
                "chunk_text": "ROS 2 is..."
            }
        ]
        selected = "User selected this: ROS fundamentals"
        prompt = rag_pipeline._build_rag_prompt(
            "Explain this",
            chunks,
            selected
        )
        assert selected in prompt
        assert "additional context" in prompt

    def test_build_rag_prompt_multiple_chunks(self, rag_pipeline):
        """Test RAG prompt with multiple chunks."""
        chunks = [
            {
                "title": "Chapter 1",
                "section": "Intro",
                "chunk_text": "ROS 2 is..."
            },
            {
                "title": "Chapter 2",
                "section": "Advanced",
                "chunk_text": "ROS 2 nodes are..."
            }
        ]
        prompt = rag_pipeline._build_rag_prompt(
            "Tell me about ROS 2",
            chunks,
            None
        )
        assert "Chapter 1" in prompt
        assert "Chapter 2" in prompt


class TestExtractSources:
    """Test source extraction from chunks."""

    def test_extract_sources_deduplication(self, rag_pipeline):
        """Test sources are deduplicated by title and URL."""
        chunks = [
            {
                "title": "Chapter 1",
                "section": "Intro",
                "url": "/docs/ch1",
                "chunk_text": "ROS 2..."
            },
            {
                "title": "Chapter 1",
                "section": "Intro",
                "url": "/docs/ch1",
                "chunk_text": "Different chunk same doc"
            },
            {
                "title": "Chapter 2",
                "section": "Advanced",
                "url": "/docs/ch2",
                "chunk_text": "ROS 2 nodes..."
            }
        ]
        sources = rag_pipeline._extract_sources(chunks)
        assert len(sources) == 2  # Only 2 unique sources

    def test_extract_sources_chunk_text_limit(self, rag_pipeline):
        """Test chunk text is limited to 200 characters."""
        chunks = [
            {
                "title": "Chapter 1",
                "section": "Intro",
                "url": "/docs/ch1",
                "chunk_text": "a" * 500
            }
        ]
        sources = rag_pipeline._extract_sources(chunks)
        assert len(sources[0].chunk_text) <= 200


class TestAnswerQuestion:
    """Test end-to-end answer generation."""

    @pytest.mark.asyncio
    async def test_answer_question_success(self, rag_pipeline):
        """Test successful end-to-end question answering."""
        with patch.object(rag_pipeline, "embed_query") as mock_embed, \
             patch.object(rag_pipeline, "search_knowledge_base") as mock_search, \
             patch.object(rag_pipeline, "retrieve_chunks_from_db") as mock_retrieve, \
             patch.object(rag_pipeline, "call_llm") as mock_llm:

            # Setup mocks
            mock_embed.return_value = [0.1] * 1536
            mock_search.return_value = [{"id": 1, "score": 0.85, "payload": {}}]
            mock_retrieve.return_value = [
                {
                    "title": "Chapter 1",
                    "section": "Intro",
                    "url": "/docs/ch1",
                    "chunk_text": "ROS 2 is..."
                }
            ]
            mock_llm.return_value = "ROS 2 is a middleware for robotics."

            answer, sources = await rag_pipeline.answer_question("What is ROS 2?")
            assert answer == "ROS 2 is a middleware for robotics."
            assert len(sources) == 1

    @pytest.mark.asyncio
    async def test_answer_question_no_results(self, rag_pipeline):
        """Test when no relevant chunks found."""
        with patch.object(rag_pipeline, "embed_query") as mock_embed, \
             patch.object(rag_pipeline, "search_knowledge_base") as mock_search:

            mock_embed.return_value = [0.1] * 1536
            mock_search.return_value = []

            answer, sources = await rag_pipeline.answer_question("Unrelated question?")
            assert "don't have information" in answer
            assert len(sources) == 0
