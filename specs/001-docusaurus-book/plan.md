# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-docusaurus-book` | **Date**: 2025-11-29 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-docusaurus-book/spec.md`

## Summary

Build a Docusaurus 3.x static site for the Physical AI & Humanoid Robotics textbook with 4-5 chapters covering ROS2, Gazebo, NVIDIA Isaac, and VLA modules. The site will be deployed to GitHub Pages with built-in search, responsive design, and dark mode support.

## Technical Context

**Language/Version**: TypeScript/JavaScript (Node.js 18+)
**Primary Dependencies**: Docusaurus 3.x, React 18, Tailwind CSS
**Storage**: N/A (static site, content in markdown files)
**Testing**: Jest for component tests, Playwright for E2E, Lighthouse for accessibility
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge), GitHub Pages hosting
**Project Type**: Web application (frontend only for this feature)
**Performance Goals**: <3s initial load, Lighthouse 90+ accessibility, <1s navigation
**Constraints**: Static site only (no SSR), GitHub Pages free tier, <100MB total size
**Scale/Scope**: 4-5 chapters, ~10,000 words total, 10-20 images

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Deadline-First Delivery | ✅ PASS | Static site is fastest to ship; Docusaurus has quick setup |
| II. Content-Before-Enhancement | ✅ PASS | This IS the content - P1 priority, no dependencies |
| III. Testable Acceptance Criteria | ✅ PASS | Spec has Given/When/Then scenarios |
| IV. Grounded AI Responses | ⬜ N/A | No AI in this feature |
| V. Accessibility-First | ✅ PASS | Lighthouse 85+ required in success criteria |
| VI. Security Boundaries | ⬜ N/A | No user data, no secrets in static site |
| VII. Spec-Driven Development | ✅ PASS | Full spec exists before planning |

**Gate Status**: ✅ PASSED - All applicable principles satisfied

## Project Structure

### Documentation (this feature)

```text
specs/001-docusaurus-book/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # N/A for static site
└── tasks.md             # Phase 2 output (/sp.tasks)
```

### Source Code (repository root)

```text
frontend/
├── docs/                    # Book chapters (MDX content)
│   ├── intro.md            # Introduction to Physical AI
│   ├── module-1-ros2/      # ROS2 Fundamentals
│   │   ├── _category_.json
│   │   ├── index.md
│   │   ├── nodes-topics.md
│   │   ├── services-actions.md
│   │   └── python-rclpy.md
│   ├── module-2-simulation/ # Robot Simulation
│   │   ├── _category_.json
│   │   ├── index.md
│   │   ├── gazebo-basics.md
│   │   └── urdf-robots.md
│   ├── module-3-nvidia-isaac/ # NVIDIA Isaac Platform
│   │   ├── _category_.json
│   │   ├── index.md
│   │   ├── isaac-sim.md
│   │   └── isaac-ros.md
│   └── module-4-vla/        # Vision-Language-Action
│       ├── _category_.json
│       ├── index.md
│       ├── voice-commands.md
│       └── llm-integration.md
├── src/
│   ├── components/          # React components
│   │   ├── CodeBlock/       # Enhanced code display
│   │   └── ImageGallery/    # Chapter images
│   ├── css/                 # Custom styles
│   │   └── custom.css
│   └── pages/               # Custom pages
│       └── index.tsx        # Landing page
├── static/
│   └── img/                 # Book images
├── docusaurus.config.ts     # Site configuration
├── sidebars.ts              # Navigation config
├── tailwind.config.js       # Tailwind setup
├── package.json
└── tsconfig.json

tests/
├── e2e/
│   ├── navigation.spec.ts   # Page navigation tests
│   └── search.spec.ts       # Search functionality tests
└── components/
    └── CodeBlock.test.tsx   # Component unit tests
```

**Structure Decision**: Web application structure with Docusaurus conventions. Using `/frontend/` for all book code to support future `/backend/` addition for RAG chatbot.

## Complexity Tracking

> No violations requiring justification - simple static site architecture.

| Area | Decision | Rationale |
|------|----------|-----------|
| Framework | Docusaurus 3.x | Purpose-built for docs, fast setup, all features built-in |
| Styling | Tailwind CSS | Utility-first, small bundle, easy customization |
| Testing | Playwright + Lighthouse | E2E for UX, Lighthouse for accessibility |
| Hosting | GitHub Pages | Free, fast, reliable, GitHub Actions integration |

