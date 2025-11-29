# Tasks: 003-podcast-generator

**Feature**: AI Podcast Generator
**Branch**: `003-podcast-generator`
**Date**: 2025-11-30
**Spec**: [spec.md](./spec.md)

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 24 |
| Setup Phase | 4 tasks |
| Backend Core | 6 tasks |
| Frontend Widget | 6 tasks |
| US1: Generate Podcast | 4 tasks |
| US2: Pre-Generated | 2 tasks |
| US3: Playback Controls | 2 tasks |
| **Completed** | **0 tasks** |
| **Remaining** | **24 tasks** |

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

- [ ] T001 Create backend podcast service directory structure
- [ ] T002 Install dependencies (Higgs Audio, GPT-4o-mini SDK, cloud storage)
- [ ] T003 Configure environment variables (API keys, storage credentials)
- [ ] T004 Set up audio file storage (S3/Cloudflare R2)

---

## Phase 2: Backend Core

> Podcast generation pipeline

- [ ] T005 Create Pydantic schemas for podcast generation request/response
- [ ] T006 Implement script generator service (GPT-4o-mini for dialogue)
- [ ] T007 Implement Higgs Audio integration for multi-speaker synthesis
- [ ] T008 Create podcast job queue service (pending/processing/completed/failed)
- [ ] T009 Implement audio file upload to cloud storage
- [ ] T010 Create podcast generation API endpoint with progress tracking

---

## Phase 3: Frontend Widget

> PodcastPlayer React component

- [ ] T011 Create PodcastPlayer component with play button
- [ ] T012 Implement audio progress bar and time display
- [ ] T013 Add playback controls (play/pause, seek, skip ±15s)
- [ ] T014 Implement mini-player that persists across navigation
- [ ] T015 Add loading indicator during generation
- [ ] T016 Integrate PodcastPlayer in chapter pages

---

## Phase 4: User Story 1 - Generate Podcast (P1)

> On-demand podcast generation

- [ ] T017 Add "Generate Podcast" button to chapter pages
- [ ] T018 Implement generation progress UI with time estimate
- [ ] T019 Handle generation queue (one per user)
- [ ] T020 Store generated podcasts for future access

---

## Phase 5: User Story 2 - Pre-Generated (P1)

> Instant playback for popular chapters

- [ ] T021 Pre-generate podcasts for main chapters (1-4)
- [ ] T022 Display "Play Podcast" button for pre-generated content

---

## Phase 6: User Story 3 - Playback Controls (P2)

> Enhanced audio controls

- [ ] T023 Implement volume control slider
- [ ] T024 Add playback speed options (0.5x, 1x, 1.5x, 2x)

---

## Phase 7: User Story 4 - Download (P3)

> Offline listening capability

- [ ] T025 Add download button for generated podcasts
- [ ] T026 Implement MP3 file download with chapter naming

---

## Phase 8: Deployment

> Production deployment

- [ ] T027 Deploy podcast service to cloud
- [ ] T028 Verify end-to-end podcast generation flow
- [ ] T029 Test mini-player persistence across pages

---

## Validation Checklist

Before marking feature complete:

- [ ] Generate Podcast button on every chapter page
- [ ] Multi-speaker audio with distinct voices
- [ ] Playback controls (play, pause, seek, skip) working
- [ ] Mini-player persists across navigation
- [ ] Pre-generated podcasts load instantly
- [ ] Download produces valid MP3
- [ ] Mobile responsive player
- [ ] Error states handled gracefully
