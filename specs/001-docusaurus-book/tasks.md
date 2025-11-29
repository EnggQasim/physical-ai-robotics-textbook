# Tasks: 001-docusaurus-book

**Feature**: Physical AI & Humanoid Robotics Textbook
**Branch**: `001-docusaurus-book`
**Date**: 2025-11-29
**Plan**: [plan.md](./plan.md) | **Spec**: [spec.md](./spec.md)

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 32 |
| Setup Phase | 6 tasks |
| Foundational Phase | 4 tasks |
| US1: Browse Content | 14 tasks |
| US2: Search | 2 tasks |
| US3: Mobile | 2 tasks |
| US4: Dark Mode | 2 tasks |
| Polish Phase | 2 tasks |
| Parallel Opportunities | 12 tasks marked [P] |

## User Story Mapping

| User Story | Priority | Tasks | Independent Test |
|------------|----------|-------|------------------|
| US1: Browse Book Content | P1 | T007-T020 | Navigate all chapters, verify content displays |
| US2: Search Within Book | P2 | T021-T022 | Type keywords, verify results link correctly |
| US3: Mobile View | P2 | T023-T024 | Test on 375px viewport, verify responsive layout |
| US4: Dark Mode | P3 | T025-T026 | Toggle theme, verify preference persists |

---

## Phase 1: Setup

> Project initialization and tooling setup

- [ ] T001 Create Docusaurus project with TypeScript template in `frontend/` directory
- [ ] T002 Install search plugin `@easyops-cn/docusaurus-search-local` in `frontend/package.json`
- [ ] T003 Install Tailwind CSS plugin `docusaurus-tailwindcss` in `frontend/package.json`
- [ ] T004 Create Tailwind configuration in `frontend/tailwind.config.js`
- [ ] T005 Create GitHub Actions workflow in `.github/workflows/deploy.yml`
- [ ] T006 Verify local development server runs at `http://localhost:3000`

---

## Phase 2: Foundational

> Blocking prerequisites for all user stories

- [ ] T007 Configure `frontend/docusaurus.config.ts` with project metadata (title, tagline, URLs)
- [ ] T008 Configure Prism syntax highlighting for Python, C++, YAML, Bash in `frontend/docusaurus.config.ts`
- [ ] T009 Create image directories `frontend/static/img/module-1/` through `module-4/`
- [ ] T010 Create chapter folder structure in `frontend/docs/` (intro, module-1-ros2, module-2-simulation, module-3-nvidia-isaac, module-4-vla)

---

## Phase 3: User Story 1 - Browse Book Content (P1)

> **Goal**: Users can navigate through all chapters and read formatted content with code examples
> **Independent Test**: Open published site, navigate all chapters, verify formatted text, code blocks, images display correctly
> **Acceptance**: FR-001, FR-002, FR-003, FR-004, FR-006, FR-010, FR-011, FR-012

### Content Structure

- [ ] T011 [P] [US1] Create introduction chapter in `frontend/docs/intro.md` with Physical AI overview (1500+ words)
- [ ] T012 [P] [US1] Create `frontend/docs/module-1-ros2/_category_.json` with label "ROS2 Fundamentals" and position 2
- [ ] T013 [P] [US1] Create `frontend/docs/module-2-simulation/_category_.json` with label "Robot Simulation" and position 3
- [ ] T014 [P] [US1] Create `frontend/docs/module-3-nvidia-isaac/_category_.json` with label "NVIDIA Isaac Platform" and position 4
- [ ] T015 [P] [US1] Create `frontend/docs/module-4-vla/_category_.json` with label "Vision-Language-Action" and position 5

### Module 1: ROS2 Fundamentals (2500+ words, 8 code examples)

- [ ] T016 [US1] Create `frontend/docs/module-1-ros2/index.md` with chapter overview and learning objectives
- [ ] T017 [US1] Create `frontend/docs/module-1-ros2/01-nodes-topics.md` with ROS2 nodes and topics explanation
- [ ] T018 [US1] Create `frontend/docs/module-1-ros2/02-services-actions.md` with ROS2 services and actions
- [ ] T019 [US1] Create `frontend/docs/module-1-ros2/03-python-rclpy.md` with Python integration examples

### Module 2: Robot Simulation (2000+ words, 5 code examples)

- [ ] T020 [US1] Create `frontend/docs/module-2-simulation/index.md` with simulation overview
- [ ] T021 [US1] Create `frontend/docs/module-2-simulation/01-gazebo-basics.md` with Gazebo introduction
- [ ] T022 [US1] Create `frontend/docs/module-2-simulation/02-urdf-robots.md` with URDF robot descriptions

### Module 3: NVIDIA Isaac (2000+ words, 4 code examples)

- [ ] T023 [US1] Create `frontend/docs/module-3-nvidia-isaac/index.md` with Isaac platform overview
- [ ] T024 [US1] Create `frontend/docs/module-3-nvidia-isaac/01-isaac-sim.md` with Isaac Sim basics
- [ ] T025 [US1] Create `frontend/docs/module-3-nvidia-isaac/02-isaac-ros.md` with Isaac ROS integration

### Module 4: Vision-Language-Action (2000+ words, 4 code examples)

- [ ] T026 [US1] Create `frontend/docs/module-4-vla/index.md` with VLA concepts overview
- [ ] T027 [US1] Create `frontend/docs/module-4-vla/01-voice-commands.md` with voice-to-action patterns
- [ ] T028 [US1] Create `frontend/docs/module-4-vla/02-llm-integration.md` with LLM robot control

---

## Phase 4: User Story 2 - Search Within Book (P2)

> **Goal**: Users can search for terms and navigate to matching sections
> **Independent Test**: Press Ctrl+K, type "ROS2 nodes", verify results appear within 2 seconds and link correctly
> **Acceptance**: FR-005, SC-004

- [ ] T029 [US2] Configure search plugin in `frontend/docusaurus.config.ts` themes array
- [ ] T030 [US2] Verify search indexes all content and returns results for "ROS2", "Gazebo", "URDF" keywords

---

## Phase 5: User Story 3 - Mobile View (P2)

> **Goal**: Site is usable on mobile devices (375px minimum)
> **Independent Test**: Open site at 375px width, verify content fits, sidebar toggles, code scrolls horizontally
> **Acceptance**: FR-008, SC-008

- [ ] T031 [US3] Add mobile responsive CSS overrides in `frontend/src/css/custom.css`
- [ ] T032 [US3] Verify mobile navigation hamburger menu works in browser responsive mode

---

## Phase 6: User Story 4 - Dark Mode (P3)

> **Goal**: Users can toggle between light and dark themes with preference persistence
> **Independent Test**: Click theme toggle, verify colors change, refresh page, verify preference remembered
> **Acceptance**: FR-009

- [ ] T033 [US4] Configure colorMode in `frontend/docusaurus.config.ts` with respectPrefersColorScheme
- [ ] T034 [US4] Verify dark mode toggle appears in navbar and persists across page refreshes

---

## Phase 7: Polish & Deployment

> Cross-cutting concerns and final verification

- [ ] T035 Run `npm run build` and verify no broken links or build errors
- [ ] T036 Deploy to GitHub Pages and verify site loads at published URL

---

## Dependencies

```text
T001 ─┬─► T002 ─┬─► T006
      ├─► T003 ─┤
      └─► T004 ─┘

T006 ──► T007 ──► T008 ──► T009 ──► T010

T010 ─┬─► T011 (intro) ────────────────────┐
      ├─► T012-T015 (categories) [parallel] │
      │                                     ▼
      └─► T016-T028 (content) ──────────► T029-T030 (search)
                                              │
                                              ▼
                                         T031-T032 (mobile)
                                              │
                                              ▼
                                         T033-T034 (dark mode)
                                              │
                                              ▼
                                         T035 ──► T036
```

## Parallel Execution Opportunities

### Setup Phase (can run in parallel after T001)
```bash
# After T001 completes, run T002, T003, T004 in parallel
T002 & T003 & T004
```

### Content Phase (can run in parallel after T010)
```bash
# Category files are independent
T012 & T013 & T014 & T015

# Module content files within same module are sequential
# But different modules can be parallel:
# Thread 1: T016 → T017 → T018 → T019 (ROS2)
# Thread 2: T020 → T021 → T022 (Simulation)
# Thread 3: T023 → T024 → T025 (Isaac)
# Thread 4: T026 → T027 → T028 (VLA)
```

---

## Implementation Strategy

### MVP Scope (Demo-Ready)
1. **Phase 1-2**: Setup + Foundational (T001-T010)
2. **Phase 3 partial**: Intro + Module 1 ROS2 only (T011, T012, T016-T019)
3. **Phase 7**: Build + Deploy (T035-T036)

This delivers a working book with 1 complete chapter for demo purposes.

### Full Scope
Complete all phases in order. Estimated: 32 tasks across 7 phases.

---

## Validation Checklist

Before marking feature complete:

- [ ] All 4 chapters have 2000+ words each
- [ ] All code examples have syntax highlighting
- [ ] All images have alt text
- [ ] Search returns results for key terms
- [ ] Site works on 375px mobile viewport
- [ ] Dark mode toggle works and persists
- [ ] Lighthouse accessibility score 85+
- [ ] Build produces no errors
- [ ] Site deployed to GitHub Pages

