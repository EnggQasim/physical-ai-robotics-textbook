# Tasks: 007-urdu-translation

**Feature**: Urdu Translation (Bilingual Support)
**Branch**: `007-urdu-translation`
**Date**: 2025-11-30
**Spec**: [spec.md](./spec.md)

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 22 |
| Setup Phase | 4 tasks |
| Backend Core | 4 tasks |
| Frontend i18n | 6 tasks |
| US1: Language Switch | 2 tasks |
| US2: Translated Content | 3 tasks |
| US3: On-Demand | 2 tasks |
| US4: Side-by-Side | 1 task |
| **Completed** | **27 tasks** |
| **Remaining** | **1 task (P3 optional)** |

## User Story Mapping

| User Story | Priority | Tasks | Independent Test |
|------------|----------|-------|------------------|
| US1: Language Switch | P1 | T005-T010 | Toggle language, verify UI changes |
| US2: Translated Content | P1 | T011-T015 | Read 3 chapters in Urdu |
| US3: On-Demand Translation | P2 | T016-T017 | Select text, get translation |
| US4: Side-by-Side | P3 | T018 | Enable side-by-side view |

---

## Phase 1: Setup

> Project initialization and Docusaurus i18n

- [X] T001 Configure Docusaurus i18n plugin for Urdu locale
- [X] T002 Add Urdu font (Jameel Noori Nastaleeq) to project
- [X] T003 Create RTL CSS styles for Urdu mode
- [X] T004 Set up interface strings file (en.json, ur.json)

---

## Phase 2: Backend Core

> Translation service implementation

- [X] T005 Create translation API endpoint (GPT-4o-mini)
- [X] T006 Implement translation caching service (in-memory)
- [X] T007 Create translated content cache (in-memory dict)
- [X] T008 API supports on-demand translation (no pre-translation needed)

---

## Phase 3: Frontend i18n

> Language switching UI

- [X] T009 Create LanguageSelector component (English/Urdu toggle)
- [X] T010 Add LanguageSelector to Docusaurus navbar
- [X] T011 Implement language preference persistence (localStorage)
- [X] T012 Apply RTL layout when Urdu is selected
- [X] T013 Translate navigation and button labels
- [X] T014 Ensure code blocks remain LTR in Urdu mode

---

## Phase 4: User Story 1 - Language Switch (P1)

> Interface language toggle

- [X] T015 Implement instant language switch without page reload
- [X] T016 Persist language preference across sessions

---

## Phase 5: User Story 2 - Translated Content (P1)

> Chapter content in Urdu

- [X] T017 Translate Module 1 (ROS2 Fundamentals) to Urdu
- [X] T018 Translate Module 2 (Robot Simulation) to Urdu
- [X] T018a Translate Module 3 (NVIDIA Isaac) to Urdu
- [X] T018b Translate Module 4 (VLA Models) to Urdu
- [X] T019 Display technical terms as "Urdu (English)" format

---

## Phase 6: User Story 3 - On-Demand (P2)

> Real-time text translation

- [X] T020 Add "Translate to Urdu" option to text selection popup
- [X] T021 Display translation in popover tooltip (TranslatePopover component)

---

## Phase 7: User Story 4 - Side-by-Side (P3)

> Bilingual comparison view

- [ ] T022 Implement side-by-side bilingual layout (desktop only)

---

## Phase 8: Deployment

> Production deployment

- [X] T023 Deploy translation service (static i18n via Docusaurus)
- [X] T024 Verify RTL layout on all pages
- [X] T025 Test language persistence on production

---

## Validation Checklist

Before marking feature complete:

- [X] Language toggle in header (English/Urdu)
- [X] Interface labels translate correctly
- [X] RTL layout renders without breaks
- [X] At least 2 chapters have Urdu translations (All 4 modules complete!)
- [X] Code blocks remain LTR in Urdu mode
- [X] Technical terms show both languages
- [X] On-demand translation works on selection (P2 - complete)
- [X] Language preference persists across sessions
