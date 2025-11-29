---
id: 001
title: Create Physical AI Textbook Constitution
stage: constitution
date: 2025-11-29
surface: agent
model: claude-opus-4-5-20250514
feature: none
branch: feature/add-constitution
user: m.qasim
command: /sp.constitution
labels: ["constitution", "hackathon", "physical-ai", "robotics", "textbook"]
links:
  spec: null
  ticket: null
  adr: null
  pr: https://github.com/EnggQasim/physical-ai-robotics-textbook/pull/1
files:
  - .specify/memory/constitution.md
  - .specify/memory/PROJECT_REFERENCE.md
tests:
  - null
---

## Prompt

User requested creation of a constitution for a hackathon project: "Physical AI & Humanoid Robotics Textbook". The project includes:
- Docusaurus book with 4-5 chapters
- Multimodal RAG Chatbot (text + image search)
- AI Podcast generation (Higgs Audio)
- AI Diagram/GIF generation (Gemini)
- User authentication (Better-Auth)
- Content personalization
- Urdu translation

Initial constitution was too detailed (included tech stack, API contracts, entities). User requested review against Panaversity documentation which showed constitutions should have TESTABLE principles only.

## Response snapshot

Created constitution v2.1.0 with:
- 7 testable core principles (Deadline-First, Content-Before-Enhancement, Testable Acceptance Criteria, Grounded AI Responses, Accessibility-First, Security Boundaries, Spec-Driven Development)
- Quality Standards with measurable metrics (response times, content quality, AI quality)
- Hard and Soft Constraints
- Development Workflow with priority order
- Definition of Done checklist
- Demo Requirements section
- Governance with amendment process

Moved implementation details (tech stack, API contracts, entities) to separate PROJECT_REFERENCE.md file.

## Outcome

- ✅ Impact: Constitution now follows Spec-Kit Plus template with testable principles
- 🧪 Tests: All principles are measurable and verifiable
- 📁 Files: constitution.md (rewritten), PROJECT_REFERENCE.md (new)
- 🔁 Next prompts: Continue with feature specifications, commit constitution
- 🧠 Reflection: Constitutions should define WHAT standards apply globally, not HOW to implement. Tech details belong in specs.

## Evaluation notes (flywheel)

- Failure modes observed: Initial constitution mixed principles with implementation details
- Graders run and results (PASS/FAIL): PASS - All principles now testable
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A - Constitution complete
