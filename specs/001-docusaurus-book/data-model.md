# Data Model: 001-docusaurus-book

**Date**: 2025-11-29 | **Plan**: [plan.md](./plan.md)

## Overview

This is a **static site** - there is no database. The "data model" describes the **content structure** stored in markdown files and configuration.

## Content Entities

### Chapter

A major section of the book stored as a folder with multiple markdown files.

**Structure**: `docs/module-N-name/`

| Field | Type | Description | Source |
|-------|------|-------------|--------|
| label | string | Display name in sidebar | `_category_.json` |
| position | number | Order in navigation (1-5) | `_category_.json` |
| description | string | Short chapter summary | `_category_.json` |
| collapsed | boolean | Sidebar default state | `_category_.json` |

**Example `_category_.json`**:
```json
{
  "label": "ROS2 Fundamentals",
  "position": 2,
  "description": "Learn the Robot Operating System 2",
  "collapsed": false
}
```

### Section

A single markdown file within a chapter folder.

**Structure**: `docs/module-N-name/NN-section-name.md`

| Field | Type | Description | Source |
|-------|------|-------------|--------|
| title | string | Page title | Frontmatter |
| sidebar_position | number | Order within chapter | Frontmatter |
| description | string | SEO/preview text | Frontmatter |
| slug | string | URL path override | Frontmatter (optional) |
| tags | string[] | Topic tags | Frontmatter (optional) |
| content | markdown | Main body text | File content |

**Example Frontmatter**:
```yaml
---
title: "ROS2 Nodes and Topics"
sidebar_position: 1
description: "Understanding the publish-subscribe pattern in ROS2"
tags: [ros2, nodes, topics, pubsub]
---
```

### CodeExample

Embedded code blocks within sections using fenced code blocks.

| Field | Type | Description | Source |
|-------|------|-------------|--------|
| language | string | Syntax highlighting | Code fence (```python) |
| code | string | Source code | Fence content |
| title | string | Display name | `title="filename.py"` |
| showLineNumbers | boolean | Line number display | Attribute |
| highlightLines | number[] | Highlighted lines | `{1-3,5}` syntax |

**Example**:
````markdown
```python title="talker.py" showLineNumbers {5-7}
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```
````

### Image

Static images stored in `static/img/` and referenced in markdown.

| Field | Type | Description | Source |
|-------|------|-------------|--------|
| src | string | File path | `static/img/chapter-N/image.png` |
| alt | string | Accessibility text | Markdown alt text |
| caption | string | Figure description | Text below image |
| width | number | Display width | HTML attribute (optional) |

**Example**:
```markdown
![ROS2 Node Communication](/img/module-1/ros2-pubsub.png)
*Figure 1.1: Publisher-Subscriber pattern in ROS2*
```

## Configuration Entities

### Site Configuration

**File**: `docusaurus.config.ts`

| Setting | Value | Purpose |
|---------|-------|---------|
| title | "Physical AI Textbook" | Browser tab, SEO |
| tagline | "Humanoid Robotics with ROS2..." | Homepage subtitle |
| url | GitHub Pages URL | Deployment target |
| baseUrl | "/" or "/repo-name/" | URL prefix |
| organizationName | GitHub username | For GH Pages |
| projectName | Repository name | For GH Pages |
| themeConfig.navbar | Navigation items | Top menu |
| themeConfig.footer | Footer links | Bottom content |
| themeConfig.colorMode | defaultMode: "light" | Theme default |

### Sidebar Configuration

**File**: `sidebars.ts`

Auto-generated from folder structure. Manual override available:

```typescript
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS2 Fundamentals',
      items: ['module-1-ros2/index', 'module-1-ros2/nodes-topics'],
    },
  ],
};
```

## Content Requirements Matrix

| Chapter | Sections | Min Words | Code Examples | Images |
|---------|----------|-----------|---------------|--------|
| Intro to Physical AI | 2 | 1500 | 2 | 3 |
| ROS2 Fundamentals | 4 | 2500 | 8 | 4 |
| Robot Simulation | 3 | 2000 | 5 | 5 |
| NVIDIA Isaac | 3 | 2000 | 4 | 4 |
| Vision-Language-Action | 3 | 2000 | 4 | 3 |
| **Total** | **15** | **10,000** | **23** | **19** |

## File Naming Conventions

| Type | Convention | Example |
|------|------------|---------|
| Chapter folder | `module-N-slug` | `module-1-ros2` |
| Section file | `NN-slug.md` | `01-nodes-topics.md` |
| Image folder | `module-N/` | `static/img/module-1/` |
| Image file | `slug-description.png` | `ros2-pubsub-diagram.png` |

## Validation Rules

1. **Frontmatter Required**: Every `.md` file must have title and sidebar_position
2. **Alt Text Required**: Every image must have alt text (accessibility)
3. **Code Language Required**: Every code block must specify language
4. **Unique Slugs**: No two pages can have the same slug
5. **Valid Links**: All internal links must resolve (build fails otherwise)

## State Transitions

N/A - Static content has no state changes.

## Relationships

```
Book
├── Chapter (1:N)
│   ├── _category_.json (1:1)
│   └── Section (1:N)
│       ├── CodeExample (0:N)
│       └── Image (0:N)
└── Config
    ├── docusaurus.config.ts (1:1)
    └── sidebars.ts (1:1)
```

