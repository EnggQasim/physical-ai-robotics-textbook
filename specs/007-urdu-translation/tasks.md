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
| **Completed** | **0 tasks** |
| **Remaining** | **22 tasks** |

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

- [ ] T001 Configure Docusaurus i18n plugin for Urdu locale
- [ ] T002 Add Urdu font (Jameel Noori Nastaleeq) to project
- [ ] T003 Create RTL CSS styles for Urdu mode
- [ ] T004 Set up interface strings file (en.json, ur.json)

---

## Phase 2: Backend Core

> Translation service implementation

- [ ] T005 Create translation API endpoint (GPT-4o-mini)
- [ ] T006 Implement translation caching service
- [ ] T007 Create translated content database table
- [ ] T008 Pre-translate main chapter content

---

## Phase 3: Frontend i18n

> Language switching UI

- [ ] T009 Create LanguageSelector component (English/Urdu toggle)
- [ ] T010 Add LanguageSelector to Docusaurus navbar
- [ ] T011 Implement language preference persistence (localStorage)
- [ ] T012 Apply RTL layout when Urdu is selected
- [ ] T013 Translate navigation and button labels
- [ ] T014 Ensure code blocks remain LTR in Urdu mode

---

## Phase 4: User Story 1 - Language Switch (P1)

> Interface language toggle

- [ ] T015 Implement instant language switch without page reload
- [ ] T016 Persist language preference across sessions

---

## Phase 5: User Story 2 - Translated Content (P1)

> Chapter content in Urdu

- [ ] T017 Translate Module 1 (ROS2 Fundamentals) to Urdu
- [ ] T018 Translate Module 2 (Robot Simulation) to Urdu
- [ ] T019 Display technical terms as "Urdu (English)" format

---

## Phase 6: User Story 3 - On-Demand (P2)

> Real-time text translation

- [ ] T020 Add "Translate to Urdu" option to text selection popup
- [ ] T021 Display translation in popover tooltip

---

## Phase 7: User Story 4 - Side-by-Side (P3)

> Bilingual comparison view

- [ ] T022 Implement side-by-side bilingual layout (desktop only)

---

## Phase 8: Deployment

> Production deployment

- [ ] T023 Deploy translation service
- [ ] T024 Verify RTL layout on all pages
- [ ] T025 Test language persistence on production

---

## Validation Checklist

Before marking feature complete:

- [ ] Language toggle in header (English/Urdu)
- [ ] Interface labels translate correctly
- [ ] RTL layout renders without breaks
- [ ] At least 2 chapters have Urdu translations
- [ ] Code blocks remain LTR in Urdu mode
- [ ] Technical terms show both languages
- [ ] On-demand translation works on selection
- [ ] Language preference persists across sessions
