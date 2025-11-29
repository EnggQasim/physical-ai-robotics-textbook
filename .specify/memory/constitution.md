# Physical AI & Humanoid Robotics Textbook - Constitution

## Project Overview

An AI-native technical textbook for teaching Physical AI & Humanoid Robotics, built with Docusaurus and deployed to GitHub Pages. The book features:
- **Multimodal RAG Chatbot** (text + image search)
- **AI Podcast Generation** (NotebookLM-style using Higgs Audio)
- **AI-Generated Diagrams & GIFs** (using Gemini)
- **User Authentication** with personalization
- **Multilingual Support** (Urdu translation)

**Hackathon Deadline**: Sunday, Nov 30, 2025 at 06:00 PM

## Core Principles

### I. MVP-First Development
- Ship working features over perfect features
- 4-5 core chapters covering essential Physical AI concepts
- Every feature must be demonstrable in 90-second demo video
- Prioritize: Core Book → RAG Chatbot → Podcast → Diagrams → Auth → Personalization → Urdu

### II. Modular Architecture
- Clear separation: Frontend (Docusaurus) / Backend (FastAPI) / AI Services
- Each component independently deployable and testable
- API-first design for all backend services
- Reusable components across features (auth, AI, i18n, media generation)

### III. AI-Native Content
- Book content optimized for RAG retrieval
- Structured markdown with semantic sections
- Images properly tagged for multimodal search
- AI-generated diagrams and animations for complex concepts
- Podcast summaries for each chapter/module
- Content chunks designed for conversational AI

### IV. Rich Media Learning
- Conceptual diagrams generated via Gemini for visual learners
- Animated GIFs for workflow explanations
- Audio podcasts for auditory learners
- Multiple modalities to enhance understanding

### V. User-Centric Personalization
- Collect user background at signup (software/hardware experience)
- Adapt content complexity based on user profile
- Support content translation (Urdu) on-demand
- Remember user preferences across sessions

### VI. Spec-Driven Development
- All features documented before implementation
- Use Spec-Kit Plus workflow: specify → plan → tasks → implement
- Create PHRs for all significant decisions
- Suggest ADRs for architectural choices

### VII. Testable Deliverables
- Each user story independently testable
- Clear acceptance criteria for all features
- Integration tests for RAG accuracy
- E2E tests for critical user flows

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
- **Monitoring**: Console logging (MVP)

## Project Structure

```text
book-project/
├── .specify/                    # Spec-Kit Plus configuration
│   ├── memory/
│   │   └── constitution.md      # This file
│   └── templates/
├── .claude/
│   ├── commands/                # Slash commands
│   ├── agents/                  # Custom subagents (bonus)
│   └── skills/                  # Reusable skills (bonus)
├── specs/                       # Feature specifications
│   ├── docusaurus-book/
│   ├── rag-chatbot/
│   ├── podcast-generator/       # NEW: Higgs Audio integration
│   ├── diagram-generator/       # NEW: Gemini diagram/GIF generation
│   ├── auth-system/
│   ├── personalization/
│   └── urdu-translation/
├── history/
│   ├── prompts/                 # PHR records
│   └── adr/                     # Architecture decisions
├── docs/                        # Docusaurus book content
│   ├── intro.md
│   ├── module-1-ros2/
│   ├── module-2-simulation/
│   ├── module-3-nvidia-isaac/
│   └── module-4-vla/
├── backend/                     # FastAPI backend
│   ├── src/
│   │   ├── api/
│   │   ├── models/
│   │   ├── services/
│   │   │   ├── rag/             # RAG service
│   │   │   ├── podcast/         # Higgs Audio service
│   │   │   ├── diagram/         # Gemini diagram service
│   │   │   ├── auth/            # Better-Auth service
│   │   │   └── personalization/ # Content adaptation
│   │   └── core/
│   └── tests/
├── frontend/                    # Docusaurus site
│   ├── docs/                    # Book chapters
│   ├── src/
│   │   ├── components/
│   │   │   ├── ChatBot/         # RAG chatbot widget
│   │   │   ├── PodcastPlayer/   # Audio player component
│   │   │   ├── DiagramViewer/   # Generated diagram display
│   │   │   └── Auth/            # Login/signup forms
│   │   ├── pages/
│   │   └── theme/
│   ├── static/
│   │   ├── audio/               # Generated podcasts
│   │   └── diagrams/            # Generated images/GIFs
│   └── docusaurus.config.js
└── scripts/                     # Utility scripts
    ├── generate-podcast.py      # Podcast generation script
    └── generate-diagrams.py     # Diagram generation script
```

## Feature Priority (Scoring Strategy)

| Priority | Feature | Points | Status |
|----------|---------|--------|--------|
| P1 | Docusaurus Book (4-5 chapters) | Base | Required |
| P1 | RAG Chatbot (text + image) | Base | Required |
| P2 | AI Podcast (Higgs Audio) | Bonus* | Differentiator |
| P2 | AI Diagrams/GIFs (Gemini) | Bonus* | Differentiator |
| P3 | Better-Auth (signup/signin) | +50 | Bonus |
| P4 | Content Personalization | +50 | Bonus |
| P5 | Urdu Translation | +50 | Bonus |
| P6 | Claude Code Agents/Skills | +50 | Bonus |

*Podcast and Diagrams are differentiators that will impress judges beyond the scoring rubric.

**Maximum Possible**: 100 (base) + 200 (bonus) + Differentiator Impact

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

## API Contracts (Core)

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

GET /api/podcast/chapter/{chapter_id}
  Response: { podcasts: Podcast[] }
```

### Diagram Generation
```
POST /api/diagram/generate
  Request: {
    concept: string,
    type: "conceptual" | "workflow" | "gif",
    chapter_id?: string
  }
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

## Quality Gates

### Code Quality
- Type hints for all Python functions
- JSDoc for React components
- No hardcoded secrets (use .env)
- Consistent naming: snake_case (Python), camelCase (JS/TS)

### Performance
- RAG response < 3 seconds
- Page load < 2 seconds
- Chatbot streaming for better UX
- Podcast generation < 60 seconds per chapter
- Diagram generation < 30 seconds
- Cache generated media (podcasts, diagrams)
- Cache translated content

### Security
- JWT tokens for authentication
- Rate limiting on AI endpoints
- Input validation on all endpoints
- CORS configured for frontend domain

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

### Content for Generation
Each chapter should specify:
```yaml
podcast:
  key_concepts: [concept1, concept2]
  target_duration: 8 # minutes

diagrams:
  - type: conceptual
    description: "ROS2 node communication architecture"
  - type: workflow
    description: "SLAM pipeline from sensor to map"
  - type: gif
    description: "Bipedal walking gait cycle animation"
```

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

## Governance

- Constitution is the source of truth for all decisions
- Amendments require explicit documentation
- All architectural decisions must be recorded as ADRs
- PHRs capture implementation history

**Version**: 1.1.0 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-11-29

## Changelog

### v1.1.0 (2025-11-29)
- Added Podcast Generation feature (Higgs Audio V2)
- Added AI Diagram/GIF Generation feature (Gemini)
- Updated project structure for new services
- Added new API contracts for podcast and diagram endpoints
- Defined AI generation guidelines
- Added new subagents and skills for content generation
