# Tasks: Mind Map & Summary Feature

**Feature**: 008-mindmap-summary
**Generated**: 2025-11-30
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Overview

Implementation tasks for adding AI-powered Summary and Mind Map tabs to each textbook chapter page.

**Task Organization**: Tasks are organized by implementation phase, with each phase aligned to a user story priority for independent testability.

---

## Phase 0: Setup & Dependencies

### Task 0.1: Install Frontend Dependencies
**Status**: [ ] Not Started
**Priority**: P0 (Blocker)
**Estimate**: 5 min

**Description**: Install React Flow and related packages for mind map visualization.

**Steps**:
1. Navigate to frontend directory
2. Install packages: `npm install reactflow html-to-image dagre @types/dagre`
3. Verify installation in package.json

**Test Cases**:
- [ ] `npm ls reactflow` shows installed version
- [ ] `npm ls html-to-image` shows installed version
- [ ] `npm ls dagre` shows installed version
- [ ] `npm run build` completes without errors

**Files**:
- `frontend/package.json` (modified)
- `frontend/package-lock.json` (modified)

---

### Task 0.2: Create Backend Directory Structure
**Status**: [ ] Not Started
**Priority**: P0 (Blocker)
**Estimate**: 2 min

**Description**: Create cache directory for mind map and summary data.

**Steps**:
1. Create `backend/mindmap/` directory for JSON cache files
2. Add `.gitkeep` to ensure directory is tracked

**Test Cases**:
- [ ] Directory `backend/mindmap/` exists
- [ ] Directory is writable

**Files**:
- `backend/mindmap/.gitkeep` (new)

---

## Phase 1: Backend Service (Foundational)

### Task 1.1: Create MindMap Service - Core Structure
**Status**: [ ] Not Started
**Priority**: P1
**Estimate**: 15 min
**Depends On**: Task 0.2

**Description**: Create the core MindMapService class with OpenAI integration and caching infrastructure.

**Steps**:
1. Create `backend/app/services/mindmap.py`
2. Implement `MindMapService` class with:
   - OpenAI client initialization
   - Cache directory setup
   - `_get_cache_key()` method for content hashing
   - `_load_cache()` and `_save_cache()` methods

**Test Cases**:
- [ ] Service instantiates without errors
- [ ] Cache file is created on first save
- [ ] Cache key is consistent for same content

**Files**:
- `backend/app/services/mindmap.py` (new)

**Code Reference** (skeleton):
```python
from pathlib import Path
import hashlib
import json
from datetime import datetime
from app.config import get_settings
import openai

class MindMapService:
    _instance = None

    def __init__(self):
        settings = get_settings()
        self.client = openai.OpenAI(api_key=settings.openai_api_key)
        self.cache_dir = Path(__file__).parent.parent.parent / "mindmap"
        self.cache_dir.mkdir(exist_ok=True)
        self.cache_file = self.cache_dir / "cache.json"
        self._cache = self._load_cache()

    def _get_cache_key(self, chapter_id: str, content: str) -> str:
        return hashlib.md5(f"{chapter_id}:{content[:500]}".encode()).hexdigest()[:12]

    def _load_cache(self) -> dict:
        if self.cache_file.exists():
            return json.loads(self.cache_file.read_text())
        return {"version": "1.0", "summaries": {}, "mindmaps": {}}

    def _save_cache(self):
        self.cache_file.write_text(json.dumps(self._cache, indent=2))

def get_mindmap_service() -> MindMapService:
    if MindMapService._instance is None:
        MindMapService._instance = MindMapService()
    return MindMapService._instance
```

---

### Task 1.2: Implement Summary Generation
**Status**: [ ] Not Started
**Priority**: P1
**Estimate**: 20 min
**Depends On**: Task 1.1

**Description**: Implement the `generate_summary()` method for AI-powered chapter summarization.

**Steps**:
1. Add `generate_summary()` async method to MindMapService
2. Implement cache-first strategy
3. Add OpenAI prompt for structured JSON output
4. Parse and validate response against Summary schema

**Test Cases**:
- [ ] Returns cached summary if available
- [ ] Generates new summary via OpenAI when not cached
- [ ] Response matches Summary schema (key_points, main_concepts, takeaways)
- [ ] Summary word count is 200-400 words
- [ ] Cache is updated after generation

**Files**:
- `backend/app/services/mindmap.py` (modified)

**Prompt Template**:
```
Analyze this chapter content and create a structured summary.

Chapter Title: {title}
Content:
{content}

Output as JSON with exactly these fields:
{
  "key_points": ["3-5 bullet points summarizing main ideas"],
  "main_concepts": [{"term": "concept name", "definition": "1-2 sentence definition"}],
  "takeaways": ["2-4 practical learning takeaways"]
}

Requirements:
- key_points: 3-5 items, each 10-50 words
- main_concepts: 3-6 items
- takeaways: 2-4 items
- Total summary should be 200-400 words
```

---

### Task 1.3: Implement Mind Map Generation
**Status**: [ ] Not Started
**Priority**: P2
**Estimate**: 25 min
**Depends On**: Task 1.1

**Description**: Implement the `generate_mindmap()` method for AI-powered concept map generation.

**Steps**:
1. Add `generate_mindmap()` async method to MindMapService
2. Implement cache-first strategy
3. Add OpenAI prompt for hierarchical node/edge structure
4. Parse and validate response against MindMap schema
5. Auto-generate edge IDs from node relationships

**Test Cases**:
- [ ] Returns cached mind map if available
- [ ] Generates new mind map via OpenAI when not cached
- [ ] Response has central_topic node at level 0
- [ ] All nodes have valid parent_id references (except central)
- [ ] Node count is between 5-25
- [ ] Max depth is 1-3 levels
- [ ] Edges correctly connect nodes

**Files**:
- `backend/app/services/mindmap.py` (modified)

**Prompt Template**:
```
Analyze this chapter and create a mind map structure.

Chapter Title: {title}
Content:
{content}

Output as JSON with this structure:
{
  "central_topic": {
    "id": "central",
    "label": "Chapter main theme (2-5 words)",
    "description": "Brief description of main topic",
    "level": 0,
    "parent_id": null
  },
  "nodes": [
    {
      "id": "node_1",
      "label": "Primary concept (2-5 words)",
      "description": "1-2 sentence explanation",
      "level": 1,
      "parent_id": "central"
    },
    {
      "id": "node_1_1",
      "label": "Sub-concept",
      "description": "Detail or example",
      "level": 2,
      "parent_id": "node_1"
    }
  ]
}

Requirements:
- 4-6 primary nodes (level 1)
- 2-3 child nodes per primary (level 2)
- Maximum 25 total nodes
- Maximum 3 levels of hierarchy
- Each label max 30 characters
- Each description max 200 characters
```

---

### Task 1.4: Create MindMap Router
**Status**: [ ] Not Started
**Priority**: P1
**Estimate**: 20 min
**Depends On**: Task 1.2, Task 1.3

**Description**: Create FastAPI router with endpoints for summary and mind map generation.

**Steps**:
1. Create `backend/app/routers/mindmap.py`
2. Define Pydantic models for request/response
3. Implement POST `/summary` endpoint
4. Implement POST `/generate` endpoint
5. Implement GET `/chapter/{chapter_id}` endpoint
6. Implement GET `/cached` endpoint
7. Add error handling with HTTPException

**Test Cases**:
- [ ] POST /api/mindmap/summary returns 200 with valid request
- [ ] POST /api/mindmap/summary returns 400 with missing fields
- [ ] POST /api/mindmap/generate returns 200 with valid request
- [ ] GET /api/mindmap/chapter/{id} returns cached data if exists
- [ ] GET /api/mindmap/chapter/{id} returns 404 if not cached
- [ ] GET /api/mindmap/cached returns list of cached items

**Files**:
- `backend/app/routers/mindmap.py` (new)

---

### Task 1.5: Register MindMap Router
**Status**: [ ] Not Started
**Priority**: P1
**Estimate**: 5 min
**Depends On**: Task 1.4

**Description**: Register the mindmap router in the FastAPI application.

**Steps**:
1. Import mindmap router in `backend/app/main.py`
2. Add `app.include_router(mindmap.router, prefix="/api")`
3. Export router in `backend/app/routers/__init__.py`

**Test Cases**:
- [ ] Backend starts without errors
- [ ] `/api/mindmap/summary` endpoint is accessible
- [ ] `/docs` shows mindmap endpoints

**Files**:
- `backend/app/main.py` (modified)
- `backend/app/routers/__init__.py` (modified)

---

## Phase 2: US1 - Summary Feature (P1)

### Task 2.1: Create SummaryPanel Component Structure
**Status**: [ ] Not Started
**Priority**: P1
**Estimate**: 15 min
**Depends On**: Task 1.5

**Description**: Create the base SummaryPanel React component with loading states.

**Steps**:
1. Create `frontend/src/components/SummaryPanel/` directory
2. Create `index.tsx` with component skeleton
3. Create `styles.module.css` with base styles
4. Implement loading and error states
5. Add fetch logic for summary API

**Test Cases**:
- [ ] Component renders without errors
- [ ] Loading spinner shown while fetching
- [ ] Error message shown on fetch failure
- [ ] Retry button works after error

**Files**:
- `frontend/src/components/SummaryPanel/index.tsx` (new)
- `frontend/src/components/SummaryPanel/styles.module.css` (new)

---

### Task 2.2: Implement Summary Display UI
**Status**: [ ] Not Started
**Priority**: P1
**Estimate**: 20 min
**Depends On**: Task 2.1

**Description**: Implement the summary content display with structured formatting.

**Steps**:
1. Create layout for key points (bullet list)
2. Create layout for main concepts (term + definition cards)
3. Create layout for takeaways (highlighted list)
4. Add "cached" indicator badge
5. Add "Regenerate" button for force refresh

**Test Cases**:
- [ ] Key points render as bullet list
- [ ] Main concepts show term and definition
- [ ] Takeaways are visually distinct
- [ ] Cached indicator shows when from cache
- [ ] Regenerate button triggers new API call with force_regenerate=true

**Files**:
- `frontend/src/components/SummaryPanel/index.tsx` (modified)
- `frontend/src/components/SummaryPanel/styles.module.css` (modified)

---

### Task 2.3: Add Summary Tab to Doc Pages
**Status**: [ ] Not Started
**Priority**: P1
**Estimate**: 25 min
**Depends On**: Task 2.2

**Description**: Integrate SummaryPanel into the DocItem layout with tab navigation.

**Steps**:
1. Swizzle DocItem Layout: `npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap`
2. Add Tabs component import from @docusaurus/theme-common
3. Wrap existing content in "Content" tab
4. Add "Summary" tab with SummaryPanel component
5. Pass chapter ID and content to SummaryPanel
6. Handle lazy loading (only fetch when tab is clicked)

**Test Cases**:
- [ ] Doc pages show tab navigation
- [ ] "Content" tab shows original doc content
- [ ] "Summary" tab shows SummaryPanel
- [ ] Tab state persists during navigation
- [ ] Summary only fetches when tab is selected

**Files**:
- `frontend/src/theme/DocItem/Layout/index.tsx` (new - swizzled)

---

### Task 2.4: Add Summary Theme Styles
**Status**: [ ] Not Started
**Priority**: P1
**Estimate**: 10 min
**Depends On**: Task 2.3

**Description**: Add styles for summary panel matching the NVIDIA-inspired theme.

**Steps**:
1. Add summary-specific CSS variables
2. Style for light/dark mode support
3. Add responsive breakpoints for mobile
4. Match existing card/panel styles

**Test Cases**:
- [ ] Summary looks correct in light mode
- [ ] Summary looks correct in dark mode
- [ ] Summary is readable on mobile devices
- [ ] Styles match existing component aesthetics

**Files**:
- `frontend/src/components/SummaryPanel/styles.module.css` (modified)
- `frontend/src/css/custom.css` (modified - if needed)

---

## Phase 3: US2 - Mind Map Feature (P2)

### Task 3.1: Create MindMapViewer Component Structure
**Status**: [ ] Not Started
**Priority**: P2
**Estimate**: 20 min
**Depends On**: Task 0.1, Task 1.5

**Description**: Create the base MindMapViewer component with React Flow integration.

**Steps**:
1. Create `frontend/src/components/MindMapViewer/` directory
2. Create `index.tsx` with React Flow setup
3. Import React Flow styles
4. Add loading and error states
5. Implement fetch logic for mind map API

**Test Cases**:
- [ ] Component renders without errors
- [ ] React Flow canvas is visible
- [ ] Loading spinner shown while fetching
- [ ] Error message with retry on failure

**Files**:
- `frontend/src/components/MindMapViewer/index.tsx` (new)
- `frontend/src/components/MindMapViewer/styles.module.css` (new)

---

### Task 3.2: Implement Dagre Layout Algorithm
**Status**: [ ] Not Started
**Priority**: P2
**Estimate**: 15 min
**Depends On**: Task 3.1

**Description**: Add automatic node positioning using Dagre layout algorithm.

**Steps**:
1. Create layout utility function using dagre
2. Convert API nodes to React Flow node format with positions
3. Set layout direction (top-to-bottom)
4. Configure node spacing and rank separation

**Test Cases**:
- [ ] Nodes are automatically positioned
- [ ] No node overlap occurs
- [ ] Central topic is at top
- [ ] Child nodes are below parents

**Files**:
- `frontend/src/components/MindMapViewer/layout.ts` (new)

**Code Reference**:
```typescript
import dagre from 'dagre';
import { Node, Edge } from 'reactflow';

const dagreGraph = new dagre.graphlib.Graph();
dagreGraph.setDefaultEdgeLabel(() => ({}));

export function getLayoutedElements(nodes: Node[], edges: Edge[]) {
  dagreGraph.setGraph({ rankdir: 'TB', nodesep: 80, ranksep: 100 });

  nodes.forEach((node) => {
    dagreGraph.setNode(node.id, { width: 150, height: 50 });
  });

  edges.forEach((edge) => {
    dagreGraph.setEdge(edge.source, edge.target);
  });

  dagre.layout(dagreGraph);

  return {
    nodes: nodes.map((node) => {
      const nodeWithPosition = dagreGraph.node(node.id);
      return {
        ...node,
        position: {
          x: nodeWithPosition.x - 75,
          y: nodeWithPosition.y - 25,
        },
      };
    }),
    edges,
  };
}
```

---

### Task 3.3: Create Custom Mind Map Node Component
**Status**: [ ] Not Started
**Priority**: P2
**Estimate**: 20 min
**Depends On**: Task 3.1

**Description**: Create custom React Flow node type with styling and tooltips.

**Steps**:
1. Create `MindMapNode.tsx` component
2. Style nodes by level (central, primary, secondary)
3. Add tooltip showing node description on hover
4. Add expand/collapse indicator for nodes with children
5. Register custom node type with React Flow

**Test Cases**:
- [ ] Central node has distinct styling
- [ ] Level 1 nodes have primary styling
- [ ] Level 2 nodes have secondary styling
- [ ] Tooltip appears on hover with description
- [ ] Expand indicator visible on nodes with children

**Files**:
- `frontend/src/components/MindMapViewer/MindMapNode.tsx` (new)
- `frontend/src/components/MindMapViewer/styles.module.css` (modified)

---

### Task 3.4: Add Mind Map Controls
**Status**: [ ] Not Started
**Priority**: P2
**Estimate**: 15 min
**Depends On**: Task 3.2, Task 3.3

**Description**: Add React Flow controls for zoom, pan, and minimap.

**Steps**:
1. Add Controls component from React Flow
2. Add MiniMap component for navigation
3. Add Background component with dots pattern
4. Configure zoom limits (0.5x to 2x)
5. Add fitView on initial load

**Test Cases**:
- [ ] Zoom in/out buttons work
- [ ] Pan by dragging canvas works
- [ ] MiniMap shows full mind map overview
- [ ] Clicking MiniMap navigates to area
- [ ] Initial view fits all nodes

**Files**:
- `frontend/src/components/MindMapViewer/index.tsx` (modified)

---

### Task 3.5: Add Mind Map Tab to Doc Pages
**Status**: [ ] Not Started
**Priority**: P2
**Estimate**: 15 min
**Depends On**: Task 3.4, Task 2.3

**Description**: Add Mind Map tab to the existing tab navigation.

**Steps**:
1. Import MindMapViewer component
2. Add "Mind Map" tab item
3. Pass chapter ID and content
4. Implement lazy loading

**Test Cases**:
- [ ] "Mind Map" tab is visible
- [ ] Clicking tab shows MindMapViewer
- [ ] Mind map renders correctly
- [ ] Mind map only fetches when tab is clicked

**Files**:
- `frontend/src/theme/DocItem/Layout/index.tsx` (modified)

---

## Phase 4: US3 - Navigation Feature (P3)

### Task 4.1: Add Content Anchor Support to Backend
**Status**: [ ] Not Started
**Priority**: P3
**Estimate**: 15 min
**Depends On**: Task 1.3

**Description**: Enhance mind map generation to include content section anchors.

**Steps**:
1. Update OpenAI prompt to extract section references
2. Add `content_anchor` field to nodes where identifiable
3. Map concepts to heading IDs in content

**Test Cases**:
- [ ] Some nodes have content_anchor populated
- [ ] Anchors match actual heading IDs in chapter
- [ ] Nodes without clear section have null anchor

**Files**:
- `backend/app/services/mindmap.py` (modified)

---

### Task 4.2: Implement Node Click Navigation
**Status**: [ ] Not Started
**Priority**: P3
**Estimate**: 20 min
**Depends On**: Task 4.1, Task 3.5

**Description**: Add double-click handler to navigate from mind map node to content section.

**Steps**:
1. Add onNodeDoubleClick handler to React Flow
2. Extract content_anchor from node data
3. Switch to Content tab
4. Scroll to anchor element with smooth behavior
5. Highlight the target section briefly

**Test Cases**:
- [ ] Double-clicking node with anchor switches to Content tab
- [ ] Page scrolls to correct section
- [ ] Section is briefly highlighted
- [ ] Nodes without anchor show tooltip message

**Files**:
- `frontend/src/components/MindMapViewer/index.tsx` (modified)
- `frontend/src/theme/DocItem/Layout/index.tsx` (modified)

---

## Phase 5: US4 - Export Feature (P4)

### Task 5.1: Implement PNG Export
**Status**: [ ] Not Started
**Priority**: P4
**Estimate**: 20 min
**Depends On**: Task 3.5

**Description**: Add export button to download mind map as PNG image.

**Steps**:
1. Add export button to MindMapViewer toolbar
2. Use html-to-image library to capture canvas
3. Generate filename: `mindmap-{chapter_id}.png`
4. Trigger browser download
5. Show loading state during export

**Test Cases**:
- [ ] Export button is visible
- [ ] Clicking export generates PNG file
- [ ] Downloaded image contains full mind map
- [ ] Text is readable in exported image
- [ ] Loading indicator shows during export

**Files**:
- `frontend/src/components/MindMapViewer/index.tsx` (modified)

**Code Reference**:
```typescript
import { toPng } from 'html-to-image';

const handleExport = async () => {
  if (!reactFlowWrapper.current) return;
  setExporting(true);
  try {
    const dataUrl = await toPng(reactFlowWrapper.current, {
      backgroundColor: '#ffffff',
      quality: 1.0,
    });
    const link = document.createElement('a');
    link.download = `mindmap-${chapterId}.png`;
    link.href = dataUrl;
    link.click();
  } finally {
    setExporting(false);
  }
};
```

---

## Phase 6: Polish & Deployment

### Task 6.1: Add Dark Mode Support
**Status**: [ ] Not Started
**Priority**: P3
**Estimate**: 15 min
**Depends On**: Task 3.5, Task 2.4

**Description**: Ensure mind map and summary components support dark theme.

**Steps**:
1. Add dark mode CSS variables for nodes
2. Update edge colors for dark mode
3. Update summary panel colors
4. Use Docusaurus theme context for detection

**Test Cases**:
- [ ] Mind map readable in dark mode
- [ ] Summary readable in dark mode
- [ ] Colors match existing dark theme
- [ ] Toggle works without reload

**Files**:
- `frontend/src/components/MindMapViewer/styles.module.css` (modified)
- `frontend/src/components/SummaryPanel/styles.module.css` (modified)

---

### Task 6.2: Add Mobile Responsive Styles
**Status**: [ ] Not Started
**Priority**: P3
**Estimate**: 15 min
**Depends On**: Task 6.1

**Description**: Ensure components work well on mobile devices.

**Steps**:
1. Add responsive breakpoints for tablets/phones
2. Adjust mind map container height on mobile
3. Simplify summary layout for narrow screens
4. Test touch interactions for zoom/pan

**Test Cases**:
- [ ] Summary readable on 375px width
- [ ] Mind map usable on 375px width
- [ ] Touch pinch-zoom works
- [ ] Touch pan works

**Files**:
- `frontend/src/components/MindMapViewer/styles.module.css` (modified)
- `frontend/src/components/SummaryPanel/styles.module.css` (modified)

---

### Task 6.3: Deploy Backend to HuggingFace
**Status**: [ ] Not Started
**Priority**: P1
**Estimate**: 10 min
**Depends On**: Task 1.5

**Description**: Upload new backend files to HuggingFace Spaces.

**Steps**:
1. Upload `backend/app/services/mindmap.py`
2. Upload `backend/app/routers/mindmap.py`
3. Update `backend/app/routers/__init__.py`
4. Update `backend/app/main.py`
5. Wait for Space to rebuild
6. Test API endpoints

**Test Cases**:
- [ ] HuggingFace Space rebuilds successfully
- [ ] `/api/mindmap/summary` returns 200
- [ ] `/api/mindmap/generate` returns 200
- [ ] No runtime errors in logs

**Files**:
- Remote HuggingFace Space files

---

### Task 6.4: Build and Deploy Frontend
**Status**: [ ] Not Started
**Priority**: P1
**Estimate**: 10 min
**Depends On**: Task 6.3, Task 3.5

**Description**: Build and push frontend changes to trigger GitHub Pages deployment.

**Steps**:
1. Run `npm run build` to verify build succeeds
2. Commit all frontend changes
3. Push to master branch
4. Monitor GitHub Actions workflow
5. Verify live site has new tabs

**Test Cases**:
- [ ] Build completes without errors
- [ ] GitHub Actions workflow succeeds
- [ ] Live site shows Summary tab
- [ ] Live site shows Mind Map tab
- [ ] Features work end-to-end

**Files**:
- All frontend files committed

---

### Task 6.5: Create PHR for Implementation
**Status**: [ ] Not Started
**Priority**: P2
**Estimate**: 5 min
**Depends On**: Task 6.4

**Description**: Create Prompt History Record for the implementation work.

**Steps**:
1. Use PHR template
2. Document all files created/modified
3. Record key decisions and outcomes
4. Add to `history/prompts/008-mindmap-summary/`

**Test Cases**:
- [ ] PHR file exists with correct ID
- [ ] All YAML placeholders filled
- [ ] Files list is complete

**Files**:
- `history/prompts/008-mindmap-summary/0003-*.prompt.md` (new)

---

## Task Summary

| Phase | Task Count | Priority | Est. Time |
|-------|------------|----------|-----------|
| Phase 0: Setup | 2 | P0 | 7 min |
| Phase 1: Backend | 5 | P1-P2 | 85 min |
| Phase 2: US1 Summary | 4 | P1 | 70 min |
| Phase 3: US2 Mind Map | 5 | P2 | 85 min |
| Phase 4: US3 Navigation | 2 | P3 | 35 min |
| Phase 5: US4 Export | 1 | P4 | 20 min |
| Phase 6: Polish | 5 | P1-P3 | 55 min |
| **Total** | **24 tasks** | | **~6 hours** |

## Critical Path

```
Task 0.1 (deps) ─────────────────────────────────────┐
                                                      ├──► Task 3.1 (MindMap component)
Task 0.2 (dirs) ──► Task 1.1 (service) ──► Task 1.2 ─┼──► Task 2.1 (Summary component)
                                       └──► Task 1.3 ─┘
                                       └──► Task 1.4 ──► Task 1.5 (router) ──► Task 6.3 (deploy)
```

## Acceptance Checklist

Before marking feature complete:

- [ ] Summary tab visible on all doc pages
- [ ] Summary generates within 10 seconds (first time)
- [ ] Summary loads within 3 seconds (cached)
- [ ] Mind Map tab visible on all doc pages
- [ ] Mind Map generates within 15 seconds (first time)
- [ ] Mind Map loads within 5 seconds (cached)
- [ ] Zoom/pan controls work
- [ ] Node tooltips display on hover
- [ ] Export to PNG works
- [ ] Dark mode supported
- [ ] Mobile responsive
- [ ] No console errors
- [ ] Backend deployed to HuggingFace
- [ ] Frontend deployed to GitHub Pages
