# Tasks: 003-podcast-generator

**Feature**: AI Podcast Generator
**Branch**: `003-podcast-generator`
**Date**: 2025-11-30
**Spec**: [spec.md](./spec.md)

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 29 |
| Setup Phase | 4 tasks |
| Backend Core | 6 tasks |
| Frontend Widget | 6 tasks |
| US1: Generate Podcast | 4 tasks |
| US2: Pre-Generated | 2 tasks |
| Higgs Audio Integration | 5 tasks |
| US3: Playback Controls | 2 tasks |
| **Completed** | **25 tasks** |
| **Remaining** | **4 tasks** |

## User Story Mapping

| User Story | Priority | Tasks | Independent Test |
|------------|----------|-------|------------------|
| US1: Generate Chapter Podcast | P1 | T005-T012 | Generate 3 chapter podcasts, verify audio quality |
| US2: Pre-Generated Podcasts | P1 | T013-T014 | Access 3 pre-generated podcasts, verify instant playback |
| US3: Playback Controls | P2 | T015-T016 | Test all controls (play, pause, seek, skip) |
| US4: Download Offline | P3 | T017-T018 | Download 2 podcasts, verify MP3 plays externally |

---

## Phase 1: Setup

> Project initialization and dependencies

- [X] T001 Create backend podcast service directory structure
- [X] T002 Install dependencies (OpenAI TTS, GPT-4o-mini SDK)
- [X] T003 Configure environment variables (API keys)
- [X] T004 Set up audio file storage (local static files)

---

## Phase 2: Backend Core

> Podcast generation pipeline

- [X] T005 Create Pydantic schemas for podcast generation request/response
- [X] T006 Implement script generator service (GPT-4o-mini for dialogue)
- [X] T007 Implement OpenAI TTS integration for audio synthesis
- [X] T008 Create podcast caching service (with metadata)
- [X] T009 Implement audio file storage and retrieval
- [X] T010 Create podcast generation API endpoint

---

## Phase 3: Frontend Widget

> PodcastPlayer React component

- [X] T011 Create PodcastPlayer component with play button
- [X] T012 Implement audio progress bar and time display
- [X] T013 Add playback controls (play/pause, seek, skip ±15s)
- [X] T014 Add volume control and playback speed options
- [X] T015 Add loading indicator during generation
- [X] T016 Add download button for podcasts

---

## Phase 4: User Story 1 - Generate Podcast (P1)

> On-demand podcast generation

- [X] T017 Add "Generate Podcast" button to component
- [X] T018 Implement generation progress UI with loading state
- [X] T019 Implement podcast caching for reuse
- [X] T020 Store generated podcasts with metadata

---

## Phase 5: User Story 2 - Pre-Generated (P1)

> Instant playback for popular chapters

- [ ] T021 Pre-generate podcasts for main chapters (1-4)
- [X] T022 Display available podcast info for chapters

---

## Phase 5.5: Higgs Audio Integration (P2)

> Multi-speaker TTS via HuggingFace Gradio API

- [X] T022a Research Higgs Audio HuggingFace integration options
- [X] T022b Create HiggsAudioService for Gradio API integration
- [X] T022c Update podcast service to support multiple TTS providers
- [X] T022d Add API endpoints for TTS provider selection (/providers, /providers/higgs/status)
- [X] T022e Implement multi-speaker audio generation (HOST/EXPERT voices)

---

## Phase 6: User Story 3 - Playback Controls (P2)

> Enhanced audio controls

- [X] T023 Implement volume control slider
- [X] T024 Add playback speed options (0.5x, 1x, 1.5x, 2x)

---

## Phase 7: User Story 4 - Download (P3)

> Offline listening capability

- [X] T025 Add download button for generated podcasts
- [X] T026 Implement MP3 file download with chapter naming

---

## Phase 8: Deployment

> Production deployment

- [ ] T027 Deploy podcast service to cloud
- [ ] T028 Verify end-to-end podcast generation flow
- [ ] T029 Test player on mobile devices

---

## Validation Checklist

Before marking feature complete:

- [X] Generate Podcast button in PodcastPlayer component
- [X] Audio generated via OpenAI TTS
- [X] Playback controls (play, pause, seek, skip) working
- [X] Volume and speed controls
- [ ] Pre-generated podcasts for chapters
- [X] Download produces valid MP3
- [X] Mobile responsive player
- [X] Error states handled gracefully
