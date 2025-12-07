#!/usr/bin/env python3
"""Document ingestion script for RAG Chatbot knowledge base.

This script:
1. Loads all markdown files from Docusaurus /docs folder
2. Extracts metadata (title, section, URL)
3. Chunks documents into 300-500 token segments
4. Embeds chunks using OpenAI API
5. Stores vectors in Qdrant
6. Stores metadata + chunk text in Neon PostgreSQL
"""

import asyncio
import argparse
import logging
import sys
from pathlib import Path
from typing import List, Dict, Any, Optional
import re
import json

import tiktoken
from openai import AsyncOpenAI

sys.path.insert(0, str(Path(__file__).parent))

from app.config import settings
from app.neon_client import neon_client
from app.qdrant_client import qdrant_client

# Setup logging
logging.basicConfig(
    level=settings.log_level,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# OpenAI client
openai_client = AsyncOpenAI(api_key=settings.openai_api_key)


# ============================================================================
# Markdown Parsing
# ============================================================================

def extract_frontmatter(content: str) -> tuple[Dict[str, str], str]:
    """Extract YAML frontmatter from markdown.

    Args:
        content: Markdown file content

    Returns:
        Tuple of (metadata dict, content without frontmatter)
    """
    if not content.startswith("---"):
        return {}, content

    try:
        # Find closing ---
        end_idx = content.find("---", 3)
        if end_idx == -1:
            return {}, content

        frontmatter_str = content[3:end_idx].strip()
        body = content[end_idx + 3:].strip()

        # Parse YAML (simple parsing, not using pyyaml)
        metadata = {}
        for line in frontmatter_str.split("\n"):
            if ":" in line:
                key, value = line.split(":", 1)
                metadata[key.strip()] = value.strip().strip('"\'')

        return metadata, body
    except Exception as e:
        logger.warning(f"Failed to parse frontmatter: {e}")
        return {}, content


def markdown_to_text(content: str) -> str:
    """Convert markdown to plain text.

    Args:
        content: Markdown content

    Returns:
        Plain text (links converted, code blocks cleaned)
    """
    # Remove code blocks
    text = re.sub(r"```[\s\S]*?```", "", content)

    # Convert markdown links [text](url) to text
    text = re.sub(r"\[([^\]]+)\]\([^\)]+\)", r"\1", text)

    # Remove other markdown syntax
    text = re.sub(r"^#+\s", "", text, flags=re.MULTILINE)  # Headers
    text = re.sub(r"[\*_]{1,3}([^\*_]+)[\*_]{1,3}", r"\1", text)  # Bold/italic
    text = re.sub(r"^[\-\*]\s", "", text, flags=re.MULTILINE)  # Lists

    return text.strip()


# ============================================================================
# Chunking
# ============================================================================

def chunk_text(text: str, min_tokens: int = 300, max_tokens: int = 500) -> List[str]:
    """Split text into chunks of min-max tokens.

    Args:
        text: Text to chunk
        min_tokens: Minimum chunk size
        max_tokens: Maximum chunk size

    Returns:
        List of text chunks
    """
    encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
    tokens = encoding.encode(text)

    chunks = []
    current_chunk = []
    current_tokens = 0

    for token in tokens:
        current_chunk.append(token)
        current_tokens += 1

        # If chunk is large enough and we hit a period or newline, flush
        if current_tokens >= max_tokens or (current_tokens >= min_tokens and token in [13, 49]):  # Period or newline
            chunk_text = encoding.decode(current_chunk)
            if chunk_text.strip():
                chunks.append(chunk_text.strip())
            current_chunk = []
            current_tokens = 0

    # Add remaining tokens
    if current_chunk:
        chunk_text = encoding.decode(current_chunk)
        if chunk_text.strip():
            chunks.append(chunk_text.strip())

    return chunks


# ============================================================================
# Embedding & Storage
# ============================================================================

async def embed_chunk(chunk: str) -> List[float]:
    """Embed a chunk using OpenAI.

    Args:
        chunk: Text chunk

    Returns:
        Embedding vector
    """
    response = await openai_client.embeddings.create(
        input=chunk,
        model=settings.openai_embedding_model,
    )
    return response.data[0].embedding


async def process_document(filepath: Path, docs_root: Path) -> tuple[int, List[str]]:
    """Process a single markdown document.

    Args:
        filepath: Path to markdown file
        docs_root: Root docs directory (for URL generation)

    Returns:
        Tuple of (document_id, list of chunk ids)
    """
    logger.info(f"Processing: {filepath.name}")

    try:
        # Read file
        with open(filepath, "r", encoding="utf-8") as f:
            content = f.read()

        # Extract metadata
        metadata, body = extract_frontmatter(content)
        title = metadata.get("title", filepath.stem)

        # Generate URL (Docusaurus convention)
        relative_path = filepath.relative_to(docs_root)
        url_path = "/" + str(relative_path).replace("\\", "/").replace(".md", "")
        url = f"/docs{url_path}"

        # Get section (folder name)
        section = relative_path.parent.name if relative_path.parent != Path(".") else "root"

        # Convert to plain text
        text = markdown_to_text(body)
        if not text:
            logger.warning(f"Empty content after parsing: {filepath}")
            return 0, []

        # Create document in Neon
        doc_id = await neon_client.create_document({
            "title": title,
            "section": section,
            "url": url,
            "content": content,  # Store original markdown
        })
        logger.info(f"Created document: {title} (ID: {doc_id})")

        # Chunk text
        chunks = chunk_text(text)
        logger.info(f"Created {len(chunks)} chunks")

        # Embed and store chunks
        chunk_ids = []
        for idx, chunk in enumerate(chunks):
            # Embed
            embedding = await embed_chunk(chunk)

            # Store in Neon
            chunk_id = await neon_client.create_chunk({
                "doc_id": doc_id,
                "chunk_text": chunk,
                "chunk_index": idx,
                "section": section,
            })
            chunk_ids.append(chunk_id)

            # Store in Qdrant
            qdrant_client.upsert_vectors([{
                "id": chunk_id,
                "embedding": embedding,
                "payload": {
                    "doc_id": doc_id,
                    "chunk_index": idx,
                    "section": section,
                }
            }])

        logger.info(f"Stored {len(chunk_ids)} chunks for {title}")
        return doc_id, chunk_ids

    except Exception as e:
        logger.error(f"Failed to process {filepath}: {e}")
        return 0, []


async def ingest_documents(docs_path: Optional[str] = None) -> Dict[str, Any]:
    """Ingest all documents from docs folder.

    Args:
        docs_path: Path to docs folder (uses DOCS_PATH env if not provided)

    Returns:
        Ingestion summary
    """
    if not docs_path:
        docs_path = settings.docs_path

    docs_root = Path(docs_path).resolve()

    if not docs_root.exists():
        logger.error(f"Docs path not found: {docs_root}")
        return {"status": "error", "error": f"Path not found: {docs_root}"}

    logger.info(f"Starting ingestion from: {docs_root}")

    # Find all markdown files
    md_files = list(docs_root.rglob("*.md"))
    logger.info(f"Found {len(md_files)} markdown files")

    if not md_files:
        logger.warning(f"No markdown files found in {docs_root}")
        return {
            "status": "success",
            "chunks_created": 0,
            "documents_processed": 0,
            "duration_seconds": 0,
            "errors": [],
        }

    # Process each file
    total_chunks = 0
    processed_docs = 0
    errors = []

    for filepath in md_files:
        try:
            doc_id, chunk_ids = await process_document(filepath, docs_root)
            if doc_id > 0:
                processed_docs += 1
                total_chunks += len(chunk_ids)
        except Exception as e:
            logger.error(f"Error processing {filepath}: {e}")
            errors.append(f"{filepath.name}: {str(e)}")

    logger.info(f"Ingestion complete: {processed_docs} docs, {total_chunks} chunks")

    return {
        "status": "success" if not errors else "partial",
        "chunks_created": total_chunks,
        "documents_processed": processed_docs,
        "duration_seconds": 0,  # Would need timing
        "errors": errors if errors else None,
    }


# ============================================================================
# CLI
# ============================================================================

async def main() -> None:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Ingest Docusaurus markdown files into RAG knowledge base"
    )
    parser.add_argument(
        "--init-db",
        action="store_true",
        help="Initialize database schema before ingestion",
    )
    parser.add_argument(
        "--docs-path",
        help="Path to docs folder (overrides DOCS_PATH env var)",
    )
    args = parser.parse_args()

    try:
        # Connect to databases
        logger.info("Connecting to databases...")
        neon_client.connect()
        qdrant_client.connect()

        # Initialize if requested
        if args.init_db:
            logger.info("Initializing database schema...")
            await neon_client.init_db()
            qdrant_client.init_collection()

        # Ingest documents
        logger.info("Starting document ingestion...")
        result = await ingest_documents(docs_path=args.docs_path)

        # Print summary
        print("\n" + "=" * 60)
        print("INGESTION SUMMARY")
        print("=" * 60)
        print(f"Status: {result['status'].upper()}")
        print(f"Documents processed: {result['documents_processed']}")
        print(f"Chunks created: {result['chunks_created']}")
        if result.get("errors"):
            print(f"Errors: {len(result['errors'])}")
            for error in result["errors"]:
                print(f"  - {error}")
        print("=" * 60 + "\n")

        # Disconnect
        await neon_client.disconnect()
        logger.info("Ingestion complete")

    except Exception as e:
        logger.error(f"Ingestion failed: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
