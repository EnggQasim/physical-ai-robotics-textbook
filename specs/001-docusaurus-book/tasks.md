# Tasks: 001-docusaurus-book

**Feature**: Physical AI & Humanoid Robotics Textbook
**Branch**: `001-docusaurus-book`
**Date**: 2025-11-29
**Plan**: [plan.md](./plan.md) | **Spec**: [spec.md](./spec.md)

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 53 |
| Setup Phase | 6 tasks (6 complete) |
| Foundational Phase | 4 tasks (4 complete) |
| US1: Browse Content | 18 tasks (18 complete) |
| US2: Search | 2 tasks (2 complete) |
| US3: Mobile | 2 tasks (2 complete) |
| US4: Dark Mode | 2 tasks (2 complete) |
| Visual Assets | 17 tasks (17 complete) |
| Polish Phase | 2 tasks (2 complete) |
| Parallel Opportunities | 28 tasks marked [P] |
| **Completed** | **53 tasks** |
| **Remaining** | **0 tasks** |

## User Story Mapping

| User Story | Priority | Tasks | Independent Test |
|------------|----------|-------|------------------|
| US1: Browse Book Content | P1 | T007-T028 | Navigate all chapters, verify content displays |
| US2: Search Within Book | P2 | T029-T030 | Type keywords, verify results link correctly |
| US3: Mobile View | P2 | T031-T032 | Test on 320px viewport, verify responsive layout |
| US4: Dark Mode | P3 | T033-T034 | Toggle theme, verify preference persists |

---

## Phase 1: Setup

> Project initialization and tooling setup

- [x] T001 Create Docusaurus project with TypeScript template in `frontend/` directory
- [x] T002 Install search plugin `@easyops-cn/docusaurus-search-local` in `frontend/package.json`
- [x] T003 Install Tailwind CSS plugin `docusaurus-tailwindcss` in `frontend/package.json`
- [x] T004 Create Tailwind configuration in `frontend/tailwind.config.js`
- [x] T005 Create GitHub Actions workflow in `.github/workflows/deploy.yml`
- [x] T006 Verify local development server runs at `http://localhost:3000`

---

## Phase 2: Foundational

> Blocking prerequisites for all user stories

- [x] T007 Configure `frontend/docusaurus.config.ts` with project metadata (title, tagline, URLs)
- [x] T008 Configure Prism syntax highlighting for Python, C++, YAML, Bash in `frontend/docusaurus.config.ts`
- [x] T009 Create image directories `frontend/static/img/module-1/` through `module-4/`
- [x] T010 Create chapter folder structure in `frontend/docs/` (intro, module-1-ros2, module-2-simulation, module-3-nvidia-isaac, module-4-vla)

---

## Phase 3: User Story 1 - Browse Book Content (P1)

> **Goal**: Users can navigate through all chapters and read formatted content with code examples
> **Independent Test**: Open published site, navigate all chapters, verify formatted text, code blocks, images display correctly
> **Acceptance**: FR-001, FR-002, FR-003, FR-004, FR-006, FR-010, FR-011, FR-012

### Content Structure

- [x] T011 [P] [US1] Create introduction chapter in `frontend/docs/intro.md` with Physical AI overview (1500+ words)
- [x] T012 [P] [US1] Create `frontend/docs/module-1-ros2/_category_.json` with label "ROS2 Fundamentals" and position 2
- [x] T013 [P] [US1] Create `frontend/docs/module-2-simulation/_category_.json` with label "Robot Simulation" and position 3
- [x] T014 [P] [US1] Create `frontend/docs/module-3-nvidia-isaac/_category_.json` with label "NVIDIA Isaac Platform" and position 4
- [x] T015 [P] [US1] Create `frontend/docs/module-4-vla/_category_.json` with label "Vision-Language-Action" and position 5

### Module 1: ROS2 Fundamentals (2500+ words, 8 code examples)

- [x] T016 [US1] Create `frontend/docs/module-1-ros2/index.md` with chapter overview and learning objectives
- [x] T017 [US1] Create `frontend/docs/module-1-ros2/01-nodes-topics.md` with ROS2 nodes and topics explanation
- [x] T018 [US1] Create `frontend/docs/module-1-ros2/02-services-actions.md` with ROS2 services and actions
- [x] T019 [US1] Create `frontend/docs/module-1-ros2/03-python-rclpy.md` with Python integration examples

### Module 2: Robot Simulation (2000+ words, 5 code examples)

- [x] T020 [US1] Create `frontend/docs/module-2-simulation/index.md` with simulation overview
- [x] T021 [US1] Create `frontend/docs/module-2-simulation/01-gazebo-basics.md` with Gazebo introduction
- [x] T022 [US1] Create `frontend/docs/module-2-simulation/02-urdf-robots.md` with URDF robot descriptions

### Module 3: NVIDIA Isaac (2000+ words, 4 code examples)

- [x] T023 [US1] Create `frontend/docs/module-3-nvidia-isaac/index.md` with Isaac platform overview
- [x] T024 [US1] Create `frontend/docs/module-3-nvidia-isaac/01-isaac-sim.md` with Isaac Sim basics
- [x] T025 [US1] Create `frontend/docs/module-3-nvidia-isaac/02-isaac-ros.md` with Isaac ROS integration

### Module 4: Vision-Language-Action (2000+ words, 4 code examples)

- [x] T026 [US1] Create `frontend/docs/module-4-vla/index.md` with VLA concepts overview
- [x] T027 [US1] Create `frontend/docs/module-4-vla/01-voice-commands.md` with voice-to-action patterns
- [x] T028 [US1] Create `frontend/docs/module-4-vla/02-llm-integration.md` with LLM robot control

---

## Phase 4: User Story 2 - Search Within Book (P2)

> **Goal**: Users can search for terms and navigate to matching sections
> **Independent Test**: Press Ctrl+K, type "ROS2 nodes", verify results appear within 2 seconds and link correctly
> **Acceptance**: FR-005, SC-004

- [x] T029 [US2] Configure search plugin in `frontend/docusaurus.config.ts` themes array
- [x] T030 [US2] Verify search indexes all content and returns results for "ROS2", "Gazebo", "URDF" keywords

---

## Phase 5: User Story 3 - Mobile View (P2)

> **Goal**: Site is usable on mobile devices (320px minimum per constitution)
> **Independent Test**: Open site at 320px width, verify content fits, sidebar toggles, code scrolls horizontally
> **Acceptance**: FR-008, SC-008

- [x] T031 [US3] Add mobile responsive CSS overrides in `frontend/src/css/custom.css`
- [x] T032 [US3] Verify mobile navigation hamburger menu works in browser responsive mode

---

## Phase 6: User Story 4 - Dark Mode (P3)

> **Goal**: Users can toggle between light and dark themes with preference persistence
> **Independent Test**: Click theme toggle, verify colors change, refresh page, verify preference remembered
> **Acceptance**: FR-009

- [x] T033 [US4] Configure colorMode in `frontend/docusaurus.config.ts` with respectPrefersColorScheme
- [x] T034 [US4] Verify dark mode toggle appears in navbar and persists across page refreshes

---

## Phase 7: Visual Assets (P2 - Differentiator)

> Generate conceptual diagrams and icons using image-generation skill

### AI-Generated Diagrams (Gemini API)

- [x] T035 [P] Generate ROS2 pub/sub diagram using `/sp.diagram ROS2 publisher subscriber communication`
- [x] T036 [P] Generate ROS2 services diagram using `/sp.diagram ROS2 service client server communication`
- [x] T037 [P] Generate Gazebo simulation architecture using `/sp.diagram --style architecture Gazebo simulation components`
- [x] T038 [P] Generate Isaac Sim platform diagram using `/sp.diagram --style architecture NVIDIA Isaac Sim platform layers`
- [x] T039 [P] Generate VLA pipeline diagram using `/sp.diagram --style workflow Voice to robot action VLA pipeline`
- [x] T040 Embed generated diagrams in corresponding chapters with alt text and captions

### Hand-Crafted SVG Icons (image-generation skill)

- [x] T043 [P] Create ROS2 module icon (`ros2-icon.svg`) showing pub/sub pattern with nodes and topic
- [x] T044 [P] Create Simulation module icon (`simulation-icon.svg`) showing 3D virtual world with robot
- [x] T045 [P] Create Isaac module icon (`isaac-icon.svg`) showing GPU chip with neural network
- [x] T046 [P] Create VLA module icon (`vla-icon.svg`) showing Vision-Language-Action cycle
- [x] T047 [P] Create Intro icon (`intro-icon.svg`) showing humanoid robot with AI brain
- [x] T048 [P] Create Industry Tools feature icon (`industry-tools-icon.svg`) showing gear + wrench + nodes
- [x] T049 [P] Create GPU Simulation feature icon (`gpu-simulation-icon.svg`) showing GPU card + 3D rendering
- [x] T050 [P] Create Cutting-Edge AI feature icon (`cutting-edge-ai-icon.svg`) showing brain + neural network
- [x] T051 Update `index.tsx` to use module icons with `useBaseUrl` hook
- [x] T052 Update `HomepageFeatures/index.tsx` to use feature icons with `useBaseUrl` hook
- [x] T053 Add hover animations for icons in CSS

---

## Phase 8: Polish & Deployment

> Cross-cutting concerns and final verification

- [x] T041 Run `npm run build` and verify no broken links or build errors
- [x] T042 Deploy to GitHub Pages and verify site loads at published URL (https://enggqasim.github.io/physical-ai-robotics-textbook/)

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
                                         T035-T039 (diagrams) [parallel]
                                              │
                                              ▼
                                         T040 (embed diagrams)
                                              │
                                              ▼
                                         T041 ──► T042
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
3. **Phase 7**: AI Diagrams (T035-T040) - at least 2 diagrams
4. **Phase 8**: Build + Deploy (T041-T042)

This delivers a working book with 1 complete chapter and AI-generated diagrams for demo purposes.

### Full Scope
Complete all phases in order. Estimated: 42 tasks across 8 phases.

---

## Validation Checklist

Before marking feature complete:

- [x] All 4 chapters have 2000+ words each
- [x] All code examples have syntax highlighting
- [x] All images have alt text
- [x] AI-generated diagrams embedded in relevant chapters
- [x] Module icons display on landing page with conceptual meaning
- [x] Feature icons display in HomepageFeatures section
- [x] Icons use `useBaseUrl` hook for proper path resolution
- [x] Icons have hover animations
- [x] Search returns results for key terms
- [x] Site works on 320px mobile viewport
- [x] Dark mode toggle works and persists
- [ ] Lighthouse accessibility score 85+
- [x] Build produces no errors
- [x] Site deployed to GitHub Pages

