# ADR-003: AI Services Stack

> **Scope**: AI and ML technology choices for RAG, podcast generation, diagram generation, and content personalization.

- **Status:** Accepted
- **Date:** 2025-11-29
- **Feature:** 002-rag-chatbot, 003-podcast-generator, 004-diagram-generator, 006-personalization
- **Context:** Need AI services to power multimodal RAG (text + image), podcast generation like NotebookLM, diagram/GIF generation, and content personalization. Must balance quality with cost.

## Decision

We will use the following AI services stack:

| Capability | Service | Model/Tier |
|------------|---------|------------|
| **Text Embeddings** | OpenAI | text-embedding-3-small (1536d) |
| **Image Embeddings** | OpenAI CLIP | clip-vit-base-patch32 (512d) |
| **Chat/RAG LLM** | OpenAI | gpt-4o-mini |
| **Personalization LLM** | OpenAI | gpt-4o |
| **Translation** | OpenAI | gpt-4o-mini |
| **Podcast Audio** | Higgs Audio V2 | Open source (Apache 2.0) |
| **Diagram Generation** | Google Gemini | gemini-pro-vision / imagen |
| **Vector Search** | Qdrant | Cosine similarity |

### Key Configuration Decisions

- **Dual embedding approach**: Text (1536d) + Image (512d) in same Qdrant collection
- **Streaming responses**: All chat endpoints stream for better UX
- **Caching strategy**: Cache embeddings, translations, and generated media
- **Rate limiting**: 10 req/min for anonymous, 30 req/min for authenticated
- **Fallback models**: gpt-4o-mini as fallback if gpt-4o quota exceeded

## Consequences

### Positive

- **Quality**: GPT-4o-mini provides excellent RAG quality at low cost
- **Multimodal**: CLIP enables image search without expensive vision models
- **Free podcast**: Higgs Audio is open source, no API costs
- **Unified API**: OpenAI SDK for most AI operations
- **Streaming**: Better perceived performance for users

### Negative

- **OpenAI dependency**: Single vendor for most AI (risk if API down)
- **Cost accumulation**: Pay-per-token can add up with heavy usage
- **Higgs Audio complexity**: Self-hosted model requires more setup
- **Gemini integration**: Separate API/SDK from OpenAI
- **CLIP limitations**: May not understand technical diagrams well

## Alternatives Considered

### Alternative A: All-OpenAI (GPT-4o Vision for images)
- **Pros**: Single API, simpler integration
- **Cons**: Very expensive for image embeddings, overkill for search
- **Why rejected**: Cost prohibitive for hackathon demo with many images

### Alternative B: Open Source Stack (Ollama + Whisper + Stable Diffusion)
- **Pros**: Free, no API limits, full control
- **Cons**: Requires GPU, complex setup, lower quality
- **Why rejected**: No GPU available, need quick setup for hackathon

### Alternative C: Anthropic Claude for RAG
- **Pros**: Longer context window, good reasoning
- **Cons**: No streaming in some SDKs, different API patterns
- **Why rejected**: OpenAI SDK more familiar, better streaming support

### Alternative D: ElevenLabs for Podcast
- **Pros**: High quality voices, easy API
- **Cons**: Expensive, not conversational format
- **Why rejected**: Higgs Audio is free and supports multi-speaker dialogue

### Alternative E: DALL-E 3 for Diagrams
- **Pros**: Same OpenAI API, high quality
- **Cons**: Expensive, not great for technical diagrams
- **Why rejected**: Gemini better for structured diagrams and workflows

## RAG Pipeline Architecture

```
User Query
    │
    ▼
┌─────────────────┐
│ Query Analysis  │ ← Determine if text/image/hybrid query
└────────┬────────┘
         │
    ┌────┴────┐
    ▼         ▼
┌───────┐ ┌───────┐
│ Text  │ │ Image │
│Embed  │ │Embed  │
│(1536d)│ │(512d) │
└───┬───┘ └───┬───┘
    │         │
    └────┬────┘
         ▼
┌─────────────────┐
│  Qdrant Search  │ ← Hybrid search with weighted scoring
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Context Assembly│ ← Top-k chunks + images
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│   GPT-4o-mini   │ ← Generate grounded response
└────────┬────────┘
         │
         ▼
    Response + Sources
```

## Cost Estimation (per 1000 queries)

| Service | Operation | Est. Cost |
|---------|-----------|-----------|
| text-embedding-3-small | 1000 queries × 100 tokens | $0.002 |
| gpt-4o-mini | 1000 responses × 500 tokens | $0.075 |
| Qdrant Cloud | Search operations | Free tier |
| Higgs Audio | Podcast generation | Free (self-hosted) |
| Gemini | Diagram generation | Free tier (60 req/min) |

**Total estimated**: ~$0.10 per 1000 RAG queries

## References

- Feature Spec: `specs/002-rag-chatbot/spec.md` (pending)
- Implementation Plan: (pending)
- Related ADRs: ADR-001 (Frontend Stack), ADR-002 (Backend Stack)
- Evaluator Evidence: N/A
