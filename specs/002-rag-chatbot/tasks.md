# Tasks: 002-rag-chatbot

**Feature**: Multimodal RAG Chatbot
**Branch**: `002-rag-chatbot`
**Date**: 2025-11-30
**Plan**: [plan.md](./plan.md) | **Spec**: [spec.md](./spec.md)

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 28 |
| Setup Phase | 4 tasks (4 complete) |
| Backend Core | 8 tasks (8 complete) |
| Frontend Widget | 6 tasks (6 complete) |
| US1: Ask Questions | 4 tasks (4 complete) |
| US2: Selected Text | 2 tasks (2 complete) |
| US3: Image Search | 2 tasks (2 complete) |
| US4: Conversation | 2 tasks (2 complete) |
| **Completed** | **28 tasks** |
| **Remaining** | **0 tasks** |

## User Story Mapping

| User Story | Priority | Tasks | Independent Test |
|------------|----------|-------|------------------|
| US1: Ask Questions | P1 | T005-T012, T017-T020 | Ask 10 questions, verify answers with sources |
| US2: Selected Text | P1 | T021-T022 | Select text, verify context-aware response |
| US3: Image Search | P2 | T023-T024 | Ask for diagrams, verify images returned |
| US4: Conversation | P3 | T025-T026 | Multi-turn conversation, verify context |

---

## Phase 1: Setup

> Project initialization and dependencies

- [x] T001 Create backend directory structure with FastAPI application
- [x] T002 Install dependencies (FastAPI, OpenAI, Qdrant, Pydantic)
- [x] T003 Configure environment variables and settings
- [x] T004 Set up Docker configuration for HuggingFace Spaces deployment

---

## Phase 2: Backend Core

> RAG pipeline implementation

- [x] T005 Create Pydantic schemas for chat messages and responses
- [x] T006 Implement OpenAI embedding service (text-embedding-3-small)
- [x] T007 Implement Qdrant vector store service (in-memory mode)
- [x] T008 Create content indexer service (markdown chunking)
- [x] T009 Implement RAG chat service with source retrieval
- [x] T010 Create health check endpoint with connectivity status
- [x] T011 Create chat API endpoint with streaming support
- [x] T012 Configure CORS for frontend integration

---

## Phase 3: Frontend Widget

> ChatWidget React component

- [x] T013 Create ChatWidget component with toggle button
- [x] T014 Implement message display with user/assistant styling
- [x] T015 Add typing indicator animation
- [x] T016 Implement source citations with clickable links
- [x] T017 Add mobile responsive CSS (375px minimum)
- [x] T018 Integrate ChatWidget in Docusaurus theme Root

---

## Phase 4: User Story 1 - Ask Questions (P1)

> Core Q&A functionality

- [x] T019 Index all markdown content on backend startup
- [x] T020 Implement RAG retrieval with relevance scoring
- [x] T021 Format responses with chapter/section citations
- [x] T022 Handle error states with user-friendly messages

---

## Phase 5: User Story 2 - Selected Text (P1)

> Context-aware questions from text selection

- [x] T023 Implement text selection detection in ChatWidget
- [x] T024 Create floating "Ask AI" button on selection

---

## Phase 6: User Story 3 - Image Search (P2)

> Multimodal image search capability

- [x] T025 [P] Implement image embeddings for diagrams (text-based semantic search)
- [x] T026 Add image search to chat responses

---

## Phase 7: User Story 4 - Conversation History (P3)

> Session persistence and context

- [x] T027 Implement in-memory conversation context (last 4 messages)
- [x] T028 Add "New Chat" button to reset conversation

---

## Phase 8: Deployment

> Production deployment

- [x] T029 Deploy backend to HuggingFace Spaces
- [x] T030 Verify API health and indexed chunks
- [x] T031 Test end-to-end chat flow on live site

---

## Validation Checklist

Before marking feature complete:

- [x] Chat widget accessible from every book page
- [x] Questions answered with source citations
- [x] Text selection triggers "Ask AI" button
- [x] Image search returns relevant diagrams
- [x] Mobile responsive (tested on 375px)
- [x] Error states handled gracefully
- [x] Backend deployed and healthy
- [x] Frontend deployed to GitHub Pages
