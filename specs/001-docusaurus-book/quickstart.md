# Quickstart: 001-docusaurus-book

**Date**: 2025-11-29 | **Plan**: [plan.md](./plan.md)

## Prerequisites

- Node.js 18+ installed (`node --version`)
- npm 9+ installed (`npm --version`)
- Git configured
- GitHub account with repo access

## Setup (5 minutes)

### 1. Clone and Navigate

```bash
cd /Users/m.qasim/Desktop/PIAIC/hackathon/book-project
```

### 2. Create Docusaurus Project

```bash
npx create-docusaurus@latest frontend classic --typescript
cd frontend
```

### 3. Install Additional Dependencies

```bash
npm install @easyops-cn/docusaurus-search-local
npm install docusaurus-tailwindcss tailwindcss postcss autoprefixer
npx tailwindcss init
```

### 4. Configure Tailwind

Create `tailwind.config.js`:
```javascript
/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ['./src/**/*.{js,jsx,ts,tsx}', './docs/**/*.{md,mdx}'],
  theme: { extend: {} },
  plugins: [],
  corePlugins: { preflight: false },
};
```

### 5. Update docusaurus.config.ts

Key changes to make:

```typescript
const config: Config = {
  title: 'Physical AI Textbook',
  tagline: 'Humanoid Robotics with ROS2, Gazebo, NVIDIA Isaac & VLA',
  url: 'https://EnggQasim.github.io',
  baseUrl: '/physical-ai-robotics-textbook/',
  organizationName: 'EnggQasim',
  projectName: 'physical-ai-robotics-textbook',

  plugins: ['docusaurus-tailwindcss'],

  themes: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        language: ['en'],
        highlightSearchTermsOnTargetPage: true,
      },
    ],
  ],

  themeConfig: {
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    prism: {
      additionalLanguages: ['python', 'cpp', 'yaml', 'bash'],
    },
  },
};
```

## Running Locally

### Development Server

```bash
cd frontend
npm start
```

Opens at http://localhost:3000 with hot reload.

### Build for Production

```bash
npm run build
```

Output in `frontend/build/` folder.

### Serve Production Build

```bash
npm run serve
```

Test production build locally at http://localhost:3000.

## Creating Content

### Add a New Chapter

1. Create folder: `docs/module-N-topic/`
2. Add category config:
```bash
cat > docs/module-1-ros2/_category_.json << 'EOF'
{
  "label": "ROS2 Fundamentals",
  "position": 2,
  "description": "Learn ROS2 basics",
  "collapsed": false
}
EOF
```

3. Add chapter intro:
```bash
cat > docs/module-1-ros2/index.md << 'EOF'
---
title: "ROS2 Fundamentals"
sidebar_position: 1
description: "Introduction to Robot Operating System 2"
---

# ROS2 Fundamentals

This chapter covers...
EOF
```

### Add a Section

```bash
cat > docs/module-1-ros2/01-nodes-topics.md << 'EOF'
---
title: "Nodes and Topics"
sidebar_position: 2
---

# ROS2 Nodes and Topics

Content here...
EOF
```

### Add Code Examples

````markdown
```python title="talker.py"
import rclpy

def main():
    rclpy.init()
    # ...
```
````

### Add Images

1. Put image in `static/img/module-1/`
2. Reference in markdown:
```markdown
![ROS2 Architecture](/img/module-1/ros2-arch.png)
```

## Deployment

### Manual Deploy

```bash
npm run build
GIT_USER=EnggQasim npm run deploy
```

### GitHub Actions (Automated)

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm
          cache-dependency-path: frontend/package-lock.json
      - name: Install dependencies
        run: npm ci
        working-directory: frontend
      - name: Build
        run: npm run build
        working-directory: frontend
      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: frontend/build

  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

## Testing

### Run Lighthouse Audit

```bash
npm install -g lighthouse
lighthouse http://localhost:3000 --output html --output-path ./lighthouse-report.html
```

### Check for Broken Links

```bash
npm run build 2>&1 | grep -i "broken"
```

### Manual Testing Checklist

- [ ] Homepage loads with navigation
- [ ] All chapters accessible from sidebar
- [ ] Search returns results
- [ ] Dark mode toggle works
- [ ] Mobile view (375px) works
- [ ] Code blocks have copy button
- [ ] All images load with alt text

## Common Issues

### "Cannot find module" Error
```bash
rm -rf node_modules package-lock.json
npm install
```

### Build Fails with Broken Links
Check `docusaurus.config.ts` for `onBrokenLinks: 'throw'` and fix the linked files.

### CSS Not Applying
Ensure Tailwind plugin is in plugins array and tailwind.config.js exists.

## Next Steps After Setup

1. ✅ Project scaffolded
2. ⬜ Create chapter folders (4 modules)
3. ⬜ Write intro.md
4. ⬜ Write Module 1: ROS2 content
5. ⬜ Add images and code examples
6. ⬜ Configure GitHub Actions
7. ⬜ Deploy to GitHub Pages

