# Book QA Agent

A complete Node.js application with a built-in web chatbot UI that answers questions from indexed book and documentation content.

## Features

- Indexes `.md`, `.mdx`, `.txt`, `.json`, and `.pdf` files recursively
- In-memory embedding index using cosine similarity
- Answers only from indexed book + docs content with citations
- Floating chat button with popup chat window
- Clean, responsive UI
- Express.js backend with REST API

## Requirements

- Node.js (v14 or higher)
- npm

## Installation

```bash
npm install
```

## Usage

```bash
npm start
```

The server will start on port 3002:
- Agent API: http://localhost:3002
- Chat UI: Click the floating chat button on any page

## API Endpoints

- `POST /ask` - Answer from indexed content
- `GET /status` - Number of indexed documents
- `GET /search?q=` - Matching text chunks

## Directory Structure

```
agent/
  agent.js          # Main application entry point
  indexer.js        # File indexing logic
  embeddings.js     # Embedding and similarity logic
  server.js         # Express server setup
  package.json      # Dependencies
  public/
    index.html      # Chat UI
    chat.js         # Chat functionality
    style.css       # Chat styling
book/               # Book content directory
docs/               # Documentation directory (already exists)
```

## How It Works

1. On startup, the agent indexes all supported files in the `book/` and `docs/` directories
2. Content is processed and stored in an in-memory embedding index
3. When a question is asked, it's matched against the indexed content using cosine similarity
4. The most relevant content chunks are used to generate an answer
5. Sources are cited with each response