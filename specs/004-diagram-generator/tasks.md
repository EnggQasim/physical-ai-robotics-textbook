# Tasks: 004-diagram-generator

**Feature**: AI Diagram & GIF Generator
**Branch**: `004-diagram-generator`
**Date**: 2025-11-30
**Spec**: [spec.md](./spec.md)

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 22 |
| Setup Phase | 3 tasks |
| Backend Core | 5 tasks |
| Frontend Widget | 5 tasks |
| US1: Concept Diagram | 3 tasks |
| US2: Animated GIF | 3 tasks |
| US3: Pre-Generated | 2 tasks |
| US4: Custom Request | 1 task |
| **Completed** | **17 tasks** |
| **Remaining** | **5 tasks** |

## User Story Mapping

| User Story | Priority | Tasks | Independent Test |
|------------|----------|-------|------------------|
| US1: Concept Diagram | P1 | T006-T011 | Generate 5 diagrams, verify accuracy |
| US2: Animated GIF | P1 | T012-T014 | Generate 3 GIFs, verify animation |
| US3: Pre-Generated | P2 | T015-T016 | View 5 chapters, verify diagrams display |
| US4: Custom Request | P3 | T017 | Request 3 custom diagrams |

---

## Phase 1: Setup

> Project initialization and dependencies

- [X] T001 Create backend diagram service directory structure
- [X] T002 Install dependencies (Gemini API, image processing libs)
- [X] T003 Configure environment variables (Gemini API key)

---

## Phase 2: Backend Core

> Diagram generation pipeline

- [X] T004 Create Pydantic schemas for diagram request/response
- [X] T005 Implement Gemini API integration for image generation
- [X] T006 Create diagram caching service (avoid regeneration)
- [ ] T007 Implement GIF generation pipeline with frame assembly
- [X] T008 Create diagram generation API endpoint

---

## Phase 3: Frontend Widget

> DiagramViewer React component

- [X] T009 Create DiagramViewer component with expand/zoom
- [X] T010 Implement lazy loading for diagrams
- [X] T011 Add generation loading indicator
- [X] T012 Create modal view for full-screen diagrams
- [X] T013 Add download button (PNG/SVG)

---

## Phase 4: User Story 1 - Concept Diagram (P1)

> AI-generated concept visualizations

- [X] T014 Add "Visualize This" button to technical sections
- [X] T015 Implement diagram generation with concept extraction
- [X] T016 Display generated diagram inline with content

---

## Phase 5: User Story 2 - Animated GIF (P1)

> Workflow and process animations

- [ ] T017 Add "Animate Workflow" button for process sections
- [ ] T018 Implement GIF generation with step progression
- [ ] T019 Add pause/play controls for GIF animation

---

## Phase 6: User Story 3 - Pre-Generated (P2)

> Embedded diagrams for key concepts

- [X] T020 Embed pre-generated diagrams in main chapters
- [X] T021 Add alt text for accessibility

---

## Phase 7: User Story 4 - Custom Request (P3)

> User-requested custom diagrams

- [ ] T022 Implement custom prompt input for diagram generation

---

## Phase 8: Deployment

> Production deployment

- [X] T023 Deploy diagram service to cloud
- [X] T024 Verify end-to-end diagram generation
- [ ] T025 Test diagram display on mobile

---

## Validation Checklist

Before marking feature complete:

- [X] "Visualize This" button on technical sections
- [X] Diagrams accurately represent concepts
- [ ] Animated GIFs show clear progression
- [X] Pre-generated diagrams display inline
- [X] Full-screen modal view works
- [X] Download produces valid PNG
- [X] Mobile responsive diagram display
- [X] Alt text for all diagrams
