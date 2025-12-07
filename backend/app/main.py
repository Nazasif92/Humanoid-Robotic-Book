"""FastAPI application for RAG Chatbot Backend."""

import logging
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

from .config import settings
from .models import AskRequest, AskResponse, Source, ErrorResponse, HealthResponse, IngestRequest, IngestResponse
from .neon_client import neon_client
from .qdrant_client import qdrant_client
from .rag_pipeline import RAGPipeline

# ============================================================================
# Setup Logging
# ============================================================================

logging.basicConfig(
    level=settings.log_level,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# ============================================================================
# Rate Limiting
# ============================================================================

limiter = Limiter(key_func=get_remote_address)


# ============================================================================
# Lifespan Events
# ============================================================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application startup and shutdown."""
    # Startup
    logger.info("Starting RAG Chatbot Backend...")
    try:
        neon_client.connect()
        qdrant_client.connect()
        qdrant_client.init_collection()
        logger.info("Database connections established")
    except Exception as e:
        logger.error(f"Failed to initialize databases: {e}")
        raise

    yield

    # Shutdown
    logger.info("Shutting down...")
    try:
        await neon_client.disconnect()
        logger.info("Disconnected from Neon")
    except Exception as e:
        logger.error(f"Error during shutdown: {e}")


# ============================================================================
# FastAPI App
# ============================================================================

app = FastAPI(
    title="RAG Chatbot Backend",
    description="Retrieval-Augmented Generation API for Humanoid Robotics Book",
    version="1.0.0",
    lifespan=lifespan,
)

# Add rate limiting
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, lambda request, exc: HTTPException(status_code=429, detail="Rate limit exceeded"))

# ============================================================================
# CORS Middleware
# ============================================================================

app.add_middleware(
    CORSMiddleware,
    allow_origins=[settings.frontend_url],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

logger.info(f"CORS enabled for: {settings.frontend_url}")

# ============================================================================
# RAG Pipeline Instance
# ============================================================================

rag_pipeline = RAGPipeline()


# ============================================================================
# Health Check Endpoint
# ============================================================================

@app.get("/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """Health check endpoint."""
    logger.info("Health check requested")

    checks = {
        "neon": await neon_client.health_check(),
        "qdrant": await qdrant_client.health_check(),
        "openai": True,  # Basic check; could enhance with actual API call
    }

    all_healthy = all(checks.values())
    status = "ok" if all_healthy else "degraded"

    logger.info(f"Health check result: {status} - {checks}")
    return HealthResponse(
        status=status,
        timestamp=time.time(),
        checks=checks,
    )


# ============================================================================
# Ask Endpoint (Main RAG Endpoint)
# ============================================================================

@app.post("/ask", response_model=AskResponse)
@limiter.limit(f"{settings.rate_limit_requests}/{settings.rate_limit_period}s")
async def ask_question(request: AskRequest, background_tasks: BackgroundTasks) -> AskResponse:
    """Answer a question using RAG pipeline.

    Args:
        request: Question + optional selected text
        background_tasks: For async logging

    Returns:
        Answer with sources and latency

    Raises:
        HTTPException: If backend error occurs
    """
    start_time = time.time()
    logger.info(f"Question received: {request.question[:50]}...")

    try:
        # Run RAG pipeline
        answer, sources = await rag_pipeline.answer_question(
            question=request.question,
            selected_text=request.selected_text,
        )

        # Calculate latency
        latency_ms = int((time.time() - start_time) * 1000)
        logger.info(f"Answer generated in {latency_ms}ms")

        # Log chat in background
        background_tasks.add_task(
            _log_chat,
            question=request.question,
            answer=answer,
            sources=[s.model_dump() for s in sources],
            latency_ms=latency_ms,
        )

        return AskResponse(
            answer=answer,
            sources=sources,
            status="success",
            latency_ms=latency_ms,
        )

    except Exception as e:
        logger.error(f"Error processing question: {e}", exc_info=True)
        latency_ms = int((time.time() - start_time) * 1000)

        # Log error in background
        background_tasks.add_task(
            _log_chat,
            question=request.question,
            error=str(e),
            latency_ms=latency_ms,
        )

        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(
                status="error",
                error="Failed to generate answer. Please try again later.",
                code="ANSWER_GENERATION_ERROR",
            ).model_dump(),
        )


# ============================================================================
# Ingest Endpoint (Internal)
# ============================================================================

@app.post("/ingest", response_model=IngestResponse)
async def ingest_documents(request: IngestRequest) -> IngestResponse:
    """Trigger document ingestion (internal endpoint).

    Args:
        request: Ingestion parameters

    Returns:
        Ingestion result with chunk count
    """
    logger.warning("Ingest endpoint called - not yet implemented")
    # TODO: Implement ingestion pipeline call
    return IngestResponse(
        status="error",
        chunks_created=0,
        documents_processed=0,
        duration_seconds=0.0,
        errors=["Ingestion endpoint not yet implemented"],
    )


# ============================================================================
# Chat History Endpoints
# ============================================================================

@app.get("/chat-history")
async def get_chat_history(limit: int = 50, offset: int = 0) -> dict:
    """Get paginated chat history.

    Args:
        limit: Number of results (max 100)
        offset: Number to skip

    Returns:
        Paginated chat history
    """
    if limit > 100:
        limit = 100

    try:
        total, chats = await neon_client.get_chat_history(limit=limit, offset=offset)
        logger.info(f"Retrieved {len(chats)} chat history items")
        return {
            "total": total,
            "limit": limit,
            "offset": offset,
            "items": chats,
        }
    except Exception as e:
        logger.error(f"Failed to retrieve chat history: {e}")
        raise HTTPException(status_code=500, detail="Failed to retrieve chat history")


@app.get("/chat-history/{chat_id}")
async def get_chat_entry(chat_id: int) -> dict:
    """Get a specific chat log entry.

    Args:
        chat_id: Chat log ID

    Returns:
        Chat log entry
    """
    try:
        chat = await neon_client.get_chat_by_id(chat_id)
        if not chat:
            raise HTTPException(status_code=404, detail="Chat entry not found")
        logger.info(f"Retrieved chat entry {chat_id}")
        return chat
    except Exception as e:
        logger.error(f"Failed to retrieve chat entry {chat_id}: {e}")
        raise HTTPException(status_code=500, detail="Failed to retrieve chat entry")


# ============================================================================
# Root Endpoint
# ============================================================================

@app.get("/")
async def root() -> dict:
    """Root endpoint."""
    return {
        "name": "RAG Chatbot Backend",
        "version": "1.0.0",
        "endpoints": {
            "health": "/health",
            "ask": "/ask (POST)",
            "ingest": "/ingest (POST)",
            "chat_history": "/chat-history (GET)",
            "chat_entry": "/chat-history/{chat_id} (GET)",
        },
    }


# ============================================================================
# Background Tasks
# ============================================================================

async def _log_chat(
    question: str,
    answer: str = None,
    sources: list = None,
    error: str = None,
    latency_ms: int = None,
) -> None:
    """Log chat interaction in background.

    Args:
        question: User question
        answer: LLM answer
        sources: Source citations
        error: Error message if failed
        latency_ms: Response latency
    """
    try:
        await neon_client.log_chat(
            {
                "question": question,
                "answer": answer,
                "sources": sources or [],
                "error": error,
                "latency_ms": latency_ms,
            }
        )
        logger.info(f"Logged chat interaction")
    except Exception as e:
        logger.error(f"Failed to log chat: {e}")


# ============================================================================
# Error Handlers
# ============================================================================

@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    """Handle HTTP exceptions."""
    logger.error(f"HTTP error: {exc.status_code} - {exc.detail}")
    return {
        "status": "error",
        "error": exc.detail if isinstance(exc.detail, str) else "An error occurred",
        "code": f"HTTP_{exc.status_code}",
    }


@app.exception_handler(Exception)
async def general_exception_handler(request, exc):
    """Handle general exceptions."""
    logger.error(f"Unexpected error: {exc}", exc_info=True)
    return {
        "status": "error",
        "error": "An unexpected error occurred",
        "code": "INTERNAL_ERROR",
    }


# ============================================================================
# Run Server
# ============================================================================

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        app,
        host=settings.backend_host,
        port=settings.backend_port,
        log_level=settings.log_level.lower(),
    )
