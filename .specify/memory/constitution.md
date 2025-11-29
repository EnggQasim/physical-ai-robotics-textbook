<!-- Sync Impact Report
Version change: 2.1.0 → 2.2.0
Modified principles: II (Content-Before-Enhancement), IV (Grounded AI Responses)
Added sections: Demo Requirements, VIII (Visual Learning with AI)
Removed sections: None
Templates requiring updates: ✅ None
Rationale: Added Gemini Nano for conceptual diagram/GIF generation to enhance learning
-->

# Physical AI Textbook Constitution

## Core Principles

### I. Deadline-First Delivery (NON-NEGOTIABLE)
All submitted work MUST be demonstrable by **November 30, 2025 at 6:00 PM PKT**.
- Features not demo-ready by deadline MUST be excluded from final submission
- Demo video MUST be under 90 seconds
- Incomplete features MUST NOT break working features
- Priority order MUST be followed: Book Content → RAG Chatbot → Bonus Features

### II. Content-Before-Enhancement
Book content MUST exist before any enhancement features begin implementation.
- Minimum 2 complete chapters MUST be written (content ready, not necessarily deployed) before RAG chatbot development starts
- Minimum 3 complete chapters MUST exist before podcast/diagram generation begins
- Each chapter MUST have: title, learning objectives, main content (2000+ words), code examples, summary
- Content structure MUST support chunking for RAG retrieval (clear headings, semantic sections)
- RAG indexing MAY proceed with draft content for development/testing purposes

### III. Testable Acceptance Criteria
Every feature MUST have measurable acceptance criteria defined BEFORE implementation begins.
- Each user story MUST include Given/When/Then scenarios
- Each requirement MUST be verifiable without subjective judgment
- Success criteria MUST use quantifiable metrics (time, count, percentage, score)
- Edge cases MUST be documented with expected behavior

### IV. Grounded AI Responses
All AI-generated responses MUST be grounded in indexed content.
- Chatbot answers MUST cite source chapter/section for every claim
- For questions directly covered in the book, chatbot MUST answer from book content only
- For related topics not explicitly in the book, chatbot MAY provide brief context while directing user to relevant chapters (e.g., "While [topic] isn't covered in detail, Chapter 3 discusses related concepts...")
- For completely unrelated topics, chatbot MUST respond: "I can only answer questions about Physical AI and Robotics topics covered in this textbook."
- Personalized content MUST preserve factual accuracy from original
- Translations MUST maintain technical term accuracy (provide original term in parentheses)

### V. Accessibility-First
All user-facing content MUST meet accessibility standards.
- Pages MUST pass Lighthouse accessibility score of 85+
- All images MUST have descriptive alt text
- All interactive elements MUST be keyboard navigable
- Color contrast MUST meet WCAG AA standards
- Mobile responsiveness MUST work on screens 320px and wider

### VI. Security Boundaries
All sensitive data MUST be protected from exposure.
- API keys and secrets MUST NEVER appear in code or version control
- All secrets MUST be loaded from environment variables
- User passwords MUST be hashed (never stored in plaintext)
- AI endpoints MUST implement rate limiting (max 10 requests/minute per IP for anonymous users)
- All user input MUST be validated before processing

### VII. Spec-Driven Development
All features MUST follow the Spec-Kit Plus workflow.
- Every feature MUST have a specification before planning begins
- Every plan MUST be approved before implementation begins
- Every significant decision MUST be recorded as PHR or ADR
- Constitution MUST be consulted when resolving conflicts between requirements

### VIII. Visual Learning with AI (Gemini API + Claude Skills)
Complex concepts SHOULD be enhanced with AI-generated visual explanations using Claude skills.

**Skills Available**:
| Skill | Location | Purpose |
|-------|----------|---------|
| image-generation | `.claude/skills/image-generation.md` | Static diagrams, icons, logos |
| generate-gif-image | `.claude/skills/generate-gif-image.md` | Animated GIFs for dynamic concepts |

**Generator Scripts**:
- `.specify/scripts/python/gemini_image.py` - SVG/PNG diagram generation
- `.specify/scripts/python/gemini_gif.py` - Animated GIF generation

#### Capabilities
- **AI-Generated Diagrams** (Gemini API): Technical diagrams, workflows, architecture
- **Hand-Crafted SVG Icons**: Module icons, feature icons, logos, branding assets
- **Animated GIFs** (Gemini 2.5 Flash): Process animations, data flows, state transitions

#### Visual Content Types
| Type | Method | Output Location |
|------|--------|-----------------|
| Technical diagrams | Gemini API | `/static/img/generated/` |
| Workflow diagrams | Gemini API | `/static/img/generated/` |
| Module icons | Hand-crafted SVG | `/static/img/icons/` |
| Feature icons | Hand-crafted SVG | `/static/img/icons/` |
| Site logo | Hand-crafted SVG | `/static/img/logo.svg` |
| Process animations | Gemini GIF API | `/static/img/animations/` |
| Data flow animations | Gemini GIF API | `/static/img/animations/` |

#### Visual Content Triggers

**Static Visuals (Diagrams/Icons)**:
- Complex algorithms or data structures
- System architectures and component interactions
- Module/feature representation (icons with conceptual meaning)
- Site branding (logo, navigation icons)

**Animated Visuals (GIFs)** - Use when showing:
- State machines and workflow sequences
- Robot motion and sensor data flows
- ROS2 communication patterns (pub/sub message flow)
- Data pipelines and processing steps
- Learning loops and training cycles
- Simulation physics steps
- Any concept where MOVEMENT helps understanding

#### Requirements
- Generated visuals MUST:
  - Include descriptive alt text for accessibility
  - Follow consistent styling (NVIDIA green theme #76b900, dark #1a1a1a)
  - Be optimized for web (SVG preferred, max 500KB for raster images)
  - Use `useBaseUrl` hook in React components for proper path resolution
- Icons MUST:
  - Be 64x64 viewBox for consistency
  - Visually represent the concept they depict
  - Include hover animations for interactivity
- Animated GIFs MUST:
  - Be max 2MB file size for fast loading
  - Loop seamlessly (end connects to start)
  - Use 10-15 fps for smooth motion
  - Be 800x600 or 600x400 dimensions
  - Have descriptive alt text explaining the animation sequence
  - Include static fallback for slow connections

#### Diagram Styles (Gemini API)
- `technical`: System components, data structures (default)
- `workflow`: Step-by-step processes
- `architecture`: High-level system design
- `simple`: Basic introductory concepts

#### Animation Styles (GIF Generation)
- `loop`: Continuous processes (e.g., ROS2 message publishing)
- `sequence`: Step-by-step procedures (e.g., robot boot sequence)
- `flow`: Data/message movement (e.g., topic message flow)
- `transition`: State changes (e.g., FSM transitions)
- `cycle`: Repeating patterns (e.g., control loop iterations)

#### Icon Design Patterns (Hand-Crafted)
- **Module Icon**: Core concept symbol + context (e.g., ROS2: nodes + topic + arrows)
- **Feature Icon**: Abstract representation (e.g., GPU: chip + 3D rendering)
- **Concept Icon**: Metaphor or analogy (e.g., AI Brain: neural network)
- **Logo**: Brand identity + concept (e.g., Robot head + neural network)

#### Visual Generation Workflow

**Decision Tree**:
1. Is the concept about MOVEMENT or FLOW? → Use **generate-gif-image** skill
2. Is it a MODULE or FEATURE icon? → Use **image-generation** skill (hand-crafted SVG)
3. Is it a static technical concept? → Use **image-generation** skill (Gemini API)

**For Static Diagrams/Icons**:
1. Run `/sp.diagram <concept>` or use image-generation skill
2. For icons: Create hand-crafted SVG with conceptual elements
3. Review for technical accuracy
4. Add to chapter with markdown: `![Alt](/img/generated/file.svg)`

**For Animated GIFs**:
1. Run `python .specify/scripts/python/gemini_gif.py --prompt "<concept>" --style <style>`
2. Review animation for clarity and smooth looping
3. Add to chapter with markdown: `![Animation: <description>](/img/animations/file.gif)`
4. Always include static fallback image

- Fallback: ASCII diagrams in code blocks when API unavailable or quota exceeded

## Quality Standards

### Response Time Requirements
| Operation | Maximum Time |
|-----------|--------------|
| Page load (initial) | 3 seconds |
| Page navigation | 1 second |
| Search results | 2 seconds |
| Chatbot response (streaming start) | 3 seconds |
| Chatbot response (complete) | 10 seconds |
| Image search results | 5 seconds |

### Content Quality Requirements
- Each chapter: minimum 2,000 words of educational content
- Code examples: must be syntactically correct and include comments
- Technical accuracy: all robotics/AI concepts must be factually correct
- Internal links: 100% must resolve (no 404 errors)

### AI Quality Requirements
- RAG relevance: 90% of answers must correctly address the question
- Source accuracy: 95% of cited sources must contain relevant information
- Hallucination rate: less than 5% of responses may contain unsupported claims

## Constraints

### Hard Constraints (Cannot Be Violated)
- Hackathon deadline: November 30, 2025, 6:00 PM PKT
- Deployment target: GitHub Pages (static site) + Backend API (separate host)
- Free tier services only: Qdrant Cloud, Neon PostgreSQL
- Demo video: maximum 90 seconds

### Soft Constraints (May Be Adjusted with Justification)
- Chapter count: 4-5 chapters (minimum 4)
- Book topic: Physical AI & Humanoid Robotics curriculum
- Language: English primary, Urdu translation as bonus

## Development Workflow

### Feature Implementation Order
1. **P1 - Required (Base 100 points)**
   - Docusaurus Book with 4-5 chapters
   - RAG Chatbot with text search
   - Image/diagram search capability

2. **P2 - Differentiators (Impress Judges)**
   - AI Podcast generation (Higgs Audio)
   - AI Diagram/GIF generation (Gemini)

3. **P3 - Bonus Features (+50 points each)**
   - User authentication (Better-Auth)
   - Content personalization
   - Urdu translation
   - Claude Code agents/skills

### Definition of Done
A feature is "done" when:
- [ ] All acceptance scenarios pass
- [ ] No console errors in browser
- [ ] Lighthouse accessibility score ≥ 85
- [ ] Works on Chrome, Firefox, Safari
- [ ] Works on mobile viewport (375px)
- [ ] Error states handled gracefully
- [ ] Documented in relevant spec file

### Demo Requirements
The final demo submission MUST meet these criteria:
- Demo video MUST be 90 seconds or less (judges will only watch first 90 seconds)
- Demo MUST show: book navigation, chatbot Q&A with source citations, at least one working bonus feature
- Demo MUST NOT require login to demonstrate core features (book + chatbot)
- Demo SHOULD have pre-cached responses for reliability (live API calls may timeout)
- Demo MUST include: GitHub repo link, deployed book URL, demo video link
- Presenter MUST be prepared to answer questions about implementation choices

## Governance

### Constitution Authority
- This Constitution supersedes all other project documentation when conflicts arise
- Feature specifications MUST NOT contradict Constitutional principles
- When in doubt, deadline-first principle takes precedence

### Amendment Process
1. Document proposed change with rationale
2. Assess impact on existing features
3. Update version number following semver:
   - MAJOR: Principle removed or fundamentally changed
   - MINOR: New principle or section added
   - PATCH: Clarification or wording improvement
4. Update LAST_AMENDED_DATE
5. Commit with message: `docs(constitution): <change summary>`

### Compliance Verification
- All PRs MUST be checked against Constitutional principles
- Spec reviews MUST verify testability of acceptance criteria
- Final submission MUST pass all Quality Standards

**Version**: 2.2.0 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-11-30
