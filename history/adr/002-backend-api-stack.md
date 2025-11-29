# ADR-002: Backend API Stack

> **Scope**: Backend technology choices for API services including framework, database, vector store, and deployment.

- **Status:** Accepted
- **Date:** 2025-11-29
- **Feature:** 002-rag-chatbot, 005-auth-system
- **Context:** Need a backend API to power RAG chatbot, authentication, personalization, and content generation features. Must work with free-tier services due to hackathon constraints.

## Decision

We will use the following backend stack:

| Component | Choice | Tier |
|-----------|--------|------|
| **Framework** | FastAPI | Python 3.11+ |
| **Database** | Neon PostgreSQL | Free tier |
| **Vector DB** | Qdrant Cloud | Free tier (1GB) |
| **Authentication** | Better-Auth | Open source |
| **Deployment** | Railway / Render | Free tier |
| **Environment** | python-dotenv | N/A |

### Key Configuration Decisions

- Async endpoints for AI operations (non-blocking)
- Pydantic models for request/response validation
- SQLAlchemy for PostgreSQL ORM
- Qdrant Python client for vector operations
- JWT tokens for session management
- CORS configured for frontend domain only

## Consequences

### Positive

- **Python ecosystem**: Best AI/ML library support (OpenAI, transformers, etc.)
- **FastAPI performance**: Async support, automatic OpenAPI docs
- **Free tier friendly**: All services have generous free tiers
- **Type safety**: Pydantic provides runtime validation
- **Rapid development**: Less boilerplate than Django/Flask
- **Serverless compatible**: Can deploy to Vercel, Railway, Render

### Negative

- **Cold starts**: Serverless may have latency on first request
- **Free tier limits**: Neon (0.5GB), Qdrant (1GB) may hit limits
- **Python GIL**: CPU-bound tasks may need async workarounds
- **No edge functions**: Can't run at edge like Next.js API routes

## Alternatives Considered

### Alternative A: Node.js + Express + Supabase
- **Pros**: JavaScript everywhere, Supabase has built-in auth and vector
- **Cons**: Less mature AI libraries, more manual setup for RAG
- **Why rejected**: Python has better AI ecosystem (OpenAI SDK, LangChain)

### Alternative B: Django + PostgreSQL + pgvector
- **Pros**: Batteries included, admin panel, ORM
- **Cons**: Heavier framework, slower development, pgvector less mature
- **Why rejected**: Overkill for hackathon, FastAPI is lighter

### Alternative C: Serverless Functions (Vercel/AWS Lambda)
- **Pros**: Auto-scaling, pay-per-use
- **Cons**: Cold starts, 10s timeout limits, complex local dev
- **Why rejected**: RAG responses may exceed timeout limits

### Alternative D: Supabase Edge Functions + Supabase Vector
- **Pros**: All-in-one platform, built-in auth
- **Cons**: Deno runtime, less Python AI library support
- **Why rejected**: Need Python for Higgs Audio and AI integrations

## Database Schema Overview

```sql
-- Users (Better-Auth managed)
users (id, email, password_hash, name, background, preferences, created_at)

-- Embeddings (Qdrant managed, metadata in Postgres)
embeddings_metadata (id, chapter_id, chunk_type, chunk_text, created_at)

-- Chat Sessions
chat_sessions (id, user_id, messages, created_at, updated_at)

-- Generated Content
podcasts (id, chapter_id, audio_url, duration, transcript, created_at)
diagrams (id, chapter_id, image_url, type, alt_text, created_at)
```

## References

- Feature Spec: `specs/002-rag-chatbot/spec.md` (pending)
- Implementation Plan: (pending)
- Related ADRs: ADR-001 (Frontend Stack), ADR-003 (AI Services Stack)
- Evaluator Evidence: N/A
