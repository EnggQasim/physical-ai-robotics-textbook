# Physical AI Textbook - Project Reference

> **Note**: This document contains implementation details extracted from the Constitution.
> These are **guidelines and decisions**, not constitutional principles.
> They may be adjusted without constitutional amendment.

## Technology Stack

### Frontend (Book)
- **Framework**: Docusaurus 3.x
- **Styling**: Tailwind CSS (via Docusaurus plugin)
- **Deployment**: GitHub Pages
- **Auth UI**: Custom React components with Better-Auth
- **I18n**: Docusaurus i18n with dynamic Urdu translation
- **Media Player**: Custom audio player for podcasts

### Backend (API)
- **Framework**: FastAPI (Python 3.11+)
- **Authentication**: Better-Auth integration
- **Database**: Neon Serverless PostgreSQL
- **Vector DB**: Qdrant Cloud (Free Tier)
- **Deployment**: Vercel/Railway/Render

### AI Services
- **Chat SDK**: OpenAI Agents SDK / ChatKit SDK
- **Embeddings**: OpenAI text-embedding-3-small (text) + CLIP (images)
- **LLM**: GPT-4o-mini for chat, GPT-4o for personalization
- **Translation**: OpenAI GPT for Urdu translation
- **Podcast**: Higgs Audio V2 (multi-speaker dialogue generation)
- **Diagrams**: Google Gemini (image generation for diagrams/GIFs)

### Infrastructure
- **Version Control**: GitHub
- **CI/CD**: GitHub Actions
- **Environment**: dotenv for secrets
- **Media Storage**: GitHub LFS or Cloudinary (for generated audio/images)

## Project Structure

```text
book-project/
├── .specify/                    # Spec-Kit Plus configuration
│   ├── memory/
│   │   ├── constitution.md      # Constitutional principles
│   │   └── PROJECT_REFERENCE.md # This file
│   └── templates/
├── .claude/
│   ├── commands/                # Slash commands
│   ├── agents/                  # Custom subagents (bonus)
│   └── skills/                  # Reusable skills (bonus)
├── specs/                       # Feature specifications
│   ├── 001-docusaurus-book/
│   ├── 002-rag-chatbot/
│   ├── 003-podcast-generator/
│   ├── 004-diagram-generator/
│   ├── 005-auth-system/
│   ├── 006-personalization/
│   └── 007-urdu-translation/
├── history/
│   ├── prompts/                 # PHR records
│   └── adr/                     # Architecture decisions
├── backend/                     # FastAPI backend
│   ├── src/
│   │   ├── api/
│   │   ├── models/
│   │   ├── services/
│   │   └── core/
│   └── tests/
├── frontend/                    # Docusaurus site
│   ├── docs/                    # Book chapters
│   ├── src/
│   │   ├── components/
│   │   ├── pages/
│   │   └── theme/
│   ├── static/
│   └── docusaurus.config.js
└── scripts/                     # Utility scripts
```

## Key Entities

### User
- id, email, password_hash
- name, background (software_level, hardware_level, prior_robotics)
- preferences (language, content_level, audio_enabled)
- created_at, updated_at

### Chapter
- id, slug, title, module
- content_markdown, content_html
- images[] (for multimodal RAG)
- podcast_url (generated audio)
- diagrams[] (generated visuals)
- reading_time, difficulty_level

### Embedding
- id, chapter_id, chunk_text, chunk_type (text/image)
- vector (1536 dimensions for text, 512 for CLIP)
- metadata (section, page, image_alt)

### Podcast
- id, chapter_id, audio_url
- duration, speakers[] (host, expert)
- transcript, generated_at

### Diagram
- id, chapter_id, image_url
- type (conceptual/workflow/gif)
- alt_text, prompt_used
- generated_at

### ChatSession
- id, user_id (nullable for anonymous)
- messages[], context_chunks[]
- created_at

## API Contracts

### RAG Chatbot
```
POST /api/chat
  Request: { message: string, session_id?: string, selected_text?: string }
  Response: { response: string, sources: ChunkRef[], session_id: string }

POST /api/chat/image-search
  Request: { query: string, image_url?: string }
  Response: { results: ImageResult[], relevance_scores: float[] }
```

### Podcast Generation
```
POST /api/podcast/generate
  Request: { chapter_id: string, style?: "conversational" | "lecture" }
  Response: { podcast_id: string, status: "processing" }

GET /api/podcast/{podcast_id}
  Response: { audio_url: string, duration: number, transcript: string }
```

### Diagram Generation
```
POST /api/diagram/generate
  Request: { concept: string, type: "conceptual" | "workflow" | "gif", chapter_id?: string }
  Response: { diagram_id: string, status: "processing" }

GET /api/diagram/{diagram_id}
  Response: { image_url: string, alt_text: string, type: string }
```

### Authentication
```
POST /api/auth/signup
  Request: { email, password, name, background: UserBackground }
  Response: { user: User, token: string }

POST /api/auth/signin
  Request: { email, password }
  Response: { user: User, token: string }
```

### Personalization
```
POST /api/personalize/chapter
  Request: { chapter_id: string, user_id: string }
  Response: { personalized_content: string, adaptations: string[] }
```

### Translation
```
POST /api/translate/urdu
  Request: { chapter_id: string, content: string }
  Response: { translated_content: string, cached: boolean }
```

## AI Generation Guidelines

### Podcast Generation (Higgs Audio)
- Two-speaker format: Host (curious learner) + Expert (teacher)
- 5-10 minutes per chapter
- Conversational, engaging tone
- Include key concepts and examples
- Store as MP3, serve via CDN

### Diagram Generation (Gemini)
- **Conceptual Diagrams**: Architecture, system overview
- **Workflow Diagrams**: Step-by-step processes (ROS2 nodes, SLAM pipeline)
- **Animated GIFs**: Motion concepts (robot kinematics, path planning)
- Use consistent visual style across book
- Generate alt-text for accessibility
- Store as PNG/GIF, optimize for web

## Reusable Intelligence (Bonus Points)

### Claude Code Subagents
1. **book-content-writer**: Generates book chapter content with proper structure
2. **rag-indexer**: Indexes content for vector search (text + images)
3. **code-example-generator**: Creates robotics code examples (Python, ROS2)
4. **podcast-script-writer**: Writes dialogue scripts for Higgs Audio
5. **diagram-prompt-engineer**: Crafts optimal prompts for Gemini diagrams

### Claude Code Skills
1. **docusaurus-page**: Creates new Docusaurus pages with frontmatter
2. **api-endpoint**: Scaffolds FastAPI endpoints with validation
3. **rag-query**: Tests RAG queries against content
4. **podcast-generate**: Triggers podcast generation for a chapter
5. **diagram-generate**: Triggers diagram generation with prompts

---

**Last Updated**: 2025-11-29
