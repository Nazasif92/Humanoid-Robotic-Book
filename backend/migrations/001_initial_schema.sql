-- Initial database schema for RAG Chatbot
-- Migration: 001_initial_schema
-- Created: 2024-01-15
-- Description: Create documents, chunks, and chat_logs tables with indexes

-- Enable required extensions
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "vector";

-- Documents table
CREATE TABLE IF NOT EXISTS documents (
  id SERIAL PRIMARY KEY,
  title VARCHAR(500) NOT NULL,
  section VARCHAR(255) NOT NULL,
  url VARCHAR(2048) NOT NULL UNIQUE,
  content TEXT NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_documents_section ON documents(section);
CREATE INDEX idx_documents_created_at ON documents(created_at DESC);
CREATE INDEX idx_documents_url ON documents(url);

-- Chunks table
CREATE TABLE IF NOT EXISTS chunks (
  id SERIAL PRIMARY KEY,
  doc_id INTEGER NOT NULL REFERENCES documents(id) ON DELETE CASCADE,
  chunk_text TEXT NOT NULL,
  chunk_index INTEGER NOT NULL,
  section VARCHAR(255) NOT NULL,
  token_count INTEGER,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_chunks_doc_id ON chunks(doc_id);
CREATE INDEX idx_chunks_section ON chunks(section);
CREATE INDEX idx_chunks_created_at ON chunks(created_at DESC);
CREATE INDEX idx_chunks_doc_chunk ON chunks(doc_id, chunk_index);
CREATE INDEX idx_chunks_metadata ON chunks USING gin(metadata);

-- Chat logs table
CREATE TABLE IF NOT EXISTS chat_logs (
  id SERIAL PRIMARY KEY,
  question VARCHAR(5000) NOT NULL,
  answer TEXT,
  sources JSONB DEFAULT '[]'::jsonb,
  error VARCHAR(500),
  latency_ms INTEGER,
  user_ip VARCHAR(45),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_chat_logs_created_at ON chat_logs(created_at DESC);
CREATE INDEX idx_chat_logs_user_ip ON chat_logs(user_ip);
CREATE INDEX idx_chat_logs_error ON chat_logs(error) WHERE error IS NOT NULL;
CREATE INDEX idx_chat_logs_metadata ON chat_logs USING gin(metadata);

-- Statistics table for monitoring
CREATE TABLE IF NOT EXISTS ingestion_logs (
  id SERIAL PRIMARY KEY,
  documents_processed INTEGER DEFAULT 0,
  chunks_created INTEGER DEFAULT 0,
  errors_count INTEGER DEFAULT 0,
  duration_seconds FLOAT,
  started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  completed_at TIMESTAMP,
  status VARCHAR(50) DEFAULT 'pending',
  metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_ingestion_logs_created_at ON ingestion_logs(started_at DESC);
CREATE INDEX idx_ingestion_logs_status ON ingestion_logs(status);

-- Create function for updating updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
  NEW.updated_at = CURRENT_TIMESTAMP;
  RETURN NEW;
END;
$$ language 'plpgsql';

-- Create trigger for documents table
CREATE TRIGGER update_documents_updated_at BEFORE UPDATE ON documents
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- Grant permissions
GRANT SELECT, INSERT, UPDATE, DELETE ON documents TO postgres;
GRANT SELECT, INSERT, UPDATE, DELETE ON chunks TO postgres;
GRANT SELECT, INSERT, UPDATE ON chat_logs TO postgres;
GRANT SELECT, INSERT ON ingestion_logs TO postgres;

-- Create a view for recent chat activity
CREATE OR REPLACE VIEW v_recent_chats AS
  SELECT
    id,
    question,
    answer,
    latency_ms,
    created_at,
    (sources @> '[{"title":""}]'::jsonb) AS has_sources,
    EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - created_at)) AS age_seconds
  FROM chat_logs
  ORDER BY created_at DESC
  LIMIT 100;
