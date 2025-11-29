# ADR-004: Multimodal RAG Architecture

> **Scope**: Architecture for Retrieval-Augmented Generation supporting both text and image search across book content.

- **Status:** Accepted
- **Date:** 2025-11-29
- **Feature:** 002-rag-chatbot
- **Context:** The textbook contains both text content and technical diagrams/images. Users should be able to ask questions and find relevant images. Need a RAG system that handles both modalities efficiently.

## Decision

We will implement a **dual-embedding multimodal RAG architecture**:

### Embedding Strategy

| Content Type | Embedding Model | Dimensions | Storage |
|--------------|-----------------|------------|---------|
| Text chunks | text-embedding-3-small | 1536 | Qdrant "text" vector |
| Images | CLIP ViT-B/32 | 512 | Qdrant "image" vector |

### Qdrant Collection Schema

```json
{
  "collection_name": "book_content",
  "vectors": {
    "text": { "size": 1536, "distance": "Cosine" },
    "image": { "size": 512, "distance": "Cosine" }
  }
}
```

### Content Indexing Strategy

1. **Text Chunking**:
   - Chunk size: 512 tokens
   - Overlap: 50 tokens
   - Preserve section boundaries (don't split mid-paragraph)
   - Store: chunk text, chapter_id, section heading, position

2. **Image Indexing**:
   - Extract all images from markdown
   - Generate CLIP embedding for each image
   - Store: image URL, alt text, caption, chapter_id

### Query Processing

```python
def process_query(query: str, query_type: str = "auto"):
    if query_type == "auto":
        query_type = detect_query_type(query)  # text/image/hybrid

    results = []

    if query_type in ["text", "hybrid"]:
        text_embedding = embed_text(query)
        text_results = qdrant.search("text", text_embedding, limit=5)
        results.extend(text_results)

    if query_type in ["image", "hybrid"]:
        image_embedding = embed_with_clip(query)
        image_results = qdrant.search("image", image_embedding, limit=3)
        results.extend(image_results)

    return dedupe_and_rank(results)
```

## Consequences

### Positive

- **Unified search**: Single query can find both text and images
- **Efficient storage**: One Qdrant collection with multiple vectors
- **Cost effective**: CLIP is free (local), only pay for text embeddings
- **Flexible queries**: Can do text-only, image-only, or hybrid search
- **Good relevance**: Cosine similarity works well for both modalities

### Negative

- **Complexity**: Two embedding models to manage
- **CLIP limitations**: May not understand domain-specific technical diagrams
- **Storage overhead**: Storing two vectors per image (CLIP + metadata)
- **Ranking challenges**: Combining scores from different embedding spaces

## Alternatives Considered

### Alternative A: Text-Only RAG (ignore images)
- **Pros**: Simpler, single embedding model
- **Cons**: Loses valuable diagram content, user explicitly requested image search
- **Why rejected**: User requirement for image search capability

### Alternative B: GPT-4o Vision for Image Understanding
- **Pros**: Better understanding of technical diagrams
- **Cons**: Expensive ($0.01+ per image), slow for search
- **Why rejected**: Cost prohibitive for indexing many images

### Alternative C: Separate Collections (text vs images)
- **Pros**: Cleaner separation, easier to manage
- **Cons**: Two searches needed, harder to combine results
- **Why rejected**: Unified collection allows hybrid queries in single call

### Alternative D: Multimodal Embedding Model (e.g., ImageBind)
- **Pros**: Single model for text and images
- **Cons**: Requires GPU, more complex setup
- **Why rejected**: No GPU available for hackathon

## Chunking Strategy Details

### Text Chunking Rules

```python
CHUNK_CONFIG = {
    "max_tokens": 512,
    "overlap_tokens": 50,
    "respect_boundaries": True,  # Don't split mid-sentence
    "separators": ["\n## ", "\n### ", "\n\n", "\n", ". "],
}
```

### Image Processing Rules

```python
IMAGE_CONFIG = {
    "supported_formats": ["png", "jpg", "jpeg", "gif", "svg"],
    "extract_from": ["markdown_images", "static_folder"],
    "require_alt_text": True,  # For accessibility
    "skip_icons": True,  # Skip small decorative images
    "min_dimensions": (100, 100),
}
```

## Retrieval Configuration

```python
RETRIEVAL_CONFIG = {
    "text_top_k": 5,
    "image_top_k": 3,
    "score_threshold": 0.7,  # Minimum relevance
    "hybrid_weight": {
        "text": 0.7,
        "image": 0.3,
    },
}
```

## References

- Feature Spec: `specs/002-rag-chatbot/spec.md` (pending)
- Implementation Plan: (pending)
- Related ADRs: ADR-003 (AI Services Stack)
- Evaluator Evidence: N/A
