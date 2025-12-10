---
id: 002
title: Retrieval Pipeline Validation Architecture & Design
stage: plan
date: 2024-01-15
surface: agent
model: claude-haiku-4-5-20251001
feature: 007-retrieval-validation
branch: 007-retrieval-validation
user: user
command: /sp.plan
labels: ["architecture", "design", "validation", "rag", "qdrant", "similarity-search"]
links:
  spec: specs/007-retrieval-validation/spec.md
  plan: specs/007-retrieval-validation/plan.md
  adr: null
  pr: null
files:
  - specs/007-retrieval-validation/plan.md
  - specs/007-retrieval-validation/tasks.md
tests:
  - Constitution Check (PASSED)
  - Architecture Review (PASSED)
  - Task Breakdown (PASSED)
---

## Prompt

Create architecture design for retrieval pipeline validation:
- Connect to Qdrant Cloud collection from Python
- Load sample queries from a test set
- Run similarity search (top-k = 5–10)
- Validate metadata integrity (url, index, path)
- Evaluate relevance using keyword or cosine threshold
- Generate retrieval_report.json (accuracy, failures, missing entries)
- Package verification script: test_retrieval.py

## Response snapshot

**Feature 007 Implementation Plan Complete**: Comprehensive architecture and technical design for Retrieval Pipeline Validation.

**Key Architectural Decisions**:
1. Qdrant async client (long-lived connection reuse across all queries)
2. Test query generation with keyword labels + cosine similarity threshold (≥0.75)
3. Metadata integrity validation (5-field comparison: url, path, title, section, chunk_index)
4. Cosine similarity classification (HIGHLY_RELEVANT ≥0.80, RELEVANT 0.70-0.79, LOW <0.70)
5. Hierarchical JSON report (summary → metrics → query results → error log)

**Technical Stack**:
- Python 3.11 with async/await patterns
- qdrant-client[http] async driver
- asyncpg for PostgreSQL metadata retrieval (reuse existing pool)
- numpy for vector operations (if needed)
- pydantic for report schema validation
- pytest-asyncio for async testing

**Architecture**:
```
test_retrieval.py (single entry point)
  ├── config.py → Load & validate environment
  ├── qdrant_client_wrapper.py → Async Qdrant operations
  ├── postgres_metadata.py → Async PostgreSQL queries
  ├── query_loader.py → Load & embed test queries
  ├── retrieval_validator.py → Core validation logic
  │   ├── validate_search_quality()
  │   ├── validate_metadata_integrity()
  │   ├── validate_chunk_ordering()
  │   └── check_missing_chunks()
  ├── similarity_evaluator.py → Relevance classification
  └── report_generator.py → JSON report output
```

**Output Contract**:
- JSON manifest: retrieval_report.json with status, summary, metrics, query results, errors
- Exit codes: 0=PASS, 1=FAIL, 2=PARTIAL, 127=config error
- Report format hierarchical: summary → metrics → query results → error log

**CLI Interface**:
```bash
python test_retrieval.py --validate-all              # Full validation
python test_retrieval.py --quick                     # 5 sample queries
python test_retrieval.py --similarity-threshold 0.70 # Set threshold
python test_retrieval.py --top-k 10                  # Set top-k results
python test_retrieval.py --output report.json        # Custom output
python test_retrieval.py -v                          # Verbose logging
```

**Implementation Phases**:
- Phase 1 (MVP): 10 tasks covering config, Qdrant/PostgreSQL clients, validation logic, similarity evaluation, reporting, CLI, tests
- Phase 2: Advanced features (incremental validation, custom metrics, CI/CD integration)
- Phase 3: Production deployment (cloud service, scheduled runs, alerting)

**Success Criteria**:
1. test_retrieval.py runs without errors
2. ≥80% top-3 search results semantically relevant
3. 100% metadata accuracy (all 5 fields match)
4. <5 min validation for 1000+ chunks
5. Comprehensive JSON report with all metrics
6. Unit + integration tests passing
7. Production-ready error handling

**Constitution Check**: ✅ PASSED - Aligns with all 5 project principles (Accuracy, Beginner-Friendly, Consistency, Example-Driven, Quality Review)

## Outcome

- ✅ Impact: Plan enables clear task generation and implementation. Establishes validation framework that ensures RAG retrieval quality before production deployment.
- 🧪 Tests: Constitution Check - PASS; Architecture Review - PASS; Task Breakdown - PASS
- 📁 Files: Generated plan.md (500+ lines) with 5 architectural decisions, detailed data model, API contracts, implementation phases; Generated tasks.md (400+ lines) with 10 executable MVP tasks with acceptance criteria
- 🔁 Next prompts: `/sp.tasks` (already executed) to generate detailed task breakdown; `/sp.implement` to begin implementation
- 🧠 Reflection: Plan balances simplicity (single test_retrieval.py script) with functionality (complete validation + reporting). Reuses Feature 006 infrastructure (Qdrant, PostgreSQL). Clear CLI contract and hierarchical report schema enable both manual testing and CI/CD automation. Metadata integrity validation (5-field comparison) catches data quality issues early.

## Evaluation notes (flywheel)

- Failure modes observed: None - specification was clear, planning requirements well-scoped
- Graders run and results (PASS/FAIL): Constitution Check - PASS; Architecture Review - PASS; Task Breakdown - PASS
- Prompt variant (if applicable): User provided explicit validation approach (Qdrant connection, similarity search top-k, metadata validation, JSON report, test_retrieval.py) which were directly incorporated into architecture decisions and task breakdown
- Next experiment (smallest change to try): Proceed directly to `/sp.implement` for Phase 1 implementation. Plan is comprehensive and ready for execution.
