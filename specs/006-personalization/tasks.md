# Tasks: 006-personalization

**Feature**: Content Personalization
**Branch**: `006-personalization`
**Date**: 2025-11-30
**Spec**: [spec.md](./spec.md)
**Dependency**: Requires 005-auth-system

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 20 |
| Setup Phase | 3 tasks |
| Backend Core | 5 tasks |
| Frontend Components | 5 tasks |
| US1: Background | 2 tasks |
| US2: Difficulty | 2 tasks |
| US3: Recommendations | 2 tasks |
| US4: Progress | 1 task |
| **Completed** | **17 tasks** |
| **Remaining** | **6 tasks** |

## User Story Mapping

| User Story | Priority | Tasks | Independent Test |
|------------|----------|-------|------------------|
| US1: Set Background | P1 | T006-T009 | Complete onboarding, verify saved |
| US2: Personalized Difficulty | P1 | T010-T013 | Compare content for different levels |
| US3: Recommendations | P2 | T014-T015 | Complete chapters, verify recommendations |
| US4: Progress Tracking | P2 | T016 | Read sections, verify progress updates |
| US5: Update Preferences | P3 | T017 | Change preferences, verify content updates |

---

## Phase 1: Setup

> Project initialization and dependencies (using in-memory storage, upgradable to PostgreSQL)

- [X] T001 Create user preferences storage (in-memory dict)
- [X] T002 Create reading progress storage (in-memory dict)
- [X] T003 Create simplified content cache (in-memory dict)

---

## Phase 2: Backend Core

> Personalization service implementation

- [X] T004 Create Pydantic schemas for preferences and progress
- [X] T005 Implement user preferences API endpoints
- [X] T006 Implement reading progress tracking API
- [X] T007 Create recommendation engine service
- [X] T008 Implement simplified content generation (GPT-4o)

---

## Phase 3: Frontend Components

> Personalization UI

- [X] T009 Create OnboardingModal component (role, level selection)
- [X] T010 Create SimplifiedToggle component for content switching
- [X] T011 Create ProgressIndicator component for sidebar
- [X] T012 Create RecommendedNext component for chapter end
- [X] T013 Add preference management to Profile page

---

## Phase 4: User Story 1 - Set Background (P1)

> First-time user onboarding

- [X] T014 Trigger onboarding modal on first sign-in
- [X] T015 Save preferences and close modal on completion

---

## Phase 5: User Story 2 - Personalized Difficulty (P1)

> Content adaptation based on level

- [ ] T016 Pre-write simplified explanations for key concepts (optional - using GPT-4o generation)
- [X] T017 Implement content switching based on user level (SimplifiedToggle component)

---

## Phase 6: User Story 3 - Recommendations (P2)

> Personalized next content suggestions

- [X] T018 Generate recommendations based on role and progress
- [X] T019 Display recommendations at chapter end (RecommendedNext component)

---

## Phase 7: User Story 4 - Progress Tracking (P2)

> Chapter completion tracking

- [X] T020 Track time on page and mark sections as viewed (ProgressIndicator component)

---

## Phase 8: Deployment

> Production deployment

- [ ] T021 Deploy personalization service
- [ ] T022 Verify onboarding flow on production
- [ ] T023 Test progress sync across devices

---

## Validation Checklist

Before marking feature complete:

- [ ] Onboarding modal appears on first sign-in
- [ ] User role and level saved correctly
- [ ] Simplified toggle switches content instantly
- [ ] Progress indicators show in sidebar
- [ ] Recommendations relevant to user role
- [ ] Preferences can be updated in settings
- [ ] Anonymous users see default content
- [ ] Progress syncs correctly
