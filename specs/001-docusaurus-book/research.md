# Research: 001-docusaurus-book

**Date**: 2025-11-29 | **Plan**: [plan.md](./plan.md)

## Research Questions

### 1. Docusaurus 3.x Setup and Configuration

**Decision**: Use Docusaurus 3.x with TypeScript template and classic theme

**Rationale**:
- Docusaurus 3 is stable and production-ready (released 2023)
- Classic theme includes search, sidebar, dark mode out of the box
- TypeScript support ensures better type safety for custom components
- Built-in MDX support allows React components in markdown

**Alternatives Considered**:
- Docusaurus 2.x: Still works but missing latest features, less maintained
- Manual React setup: Much more work, no built-in docs features

**Key Setup Commands**:
```bash
npx create-docusaurus@latest frontend classic --typescript
cd frontend && npm install
npm start  # Dev server at localhost:3000
npm run build  # Static build for deployment
```

### 2. Local Search Plugin

**Decision**: Use `@easyops-cn/docusaurus-search-local` plugin

**Rationale**:
- No external service required (Algolia needs API key setup)
- Works offline after initial page load
- Indexes on build time, fast search at runtime
- Free and open source

**Alternatives Considered**:
- Algolia DocSearch: Better search quality but requires API key and approval
- Lunr.js manual setup: More control but more setup work

**Configuration**:
```js
// docusaurus.config.js
themes: [
  [
    require.resolve("@easyops-cn/docusaurus-search-local"),
    {
      hashed: true,
      language: ["en"],
      highlightSearchTermsOnTargetPage: true,
    },
  ],
],
```

### 3. Tailwind CSS Integration

**Decision**: Use `docusaurus-tailwindcss` plugin for Tailwind v3 integration

**Rationale**:
- Official community plugin, well-maintained
- Doesn't conflict with Docusaurus default styles
- Allows utility classes in custom components
- Tree-shaking removes unused CSS

**Alternatives Considered**:
- Infima only (default): Limited customization, harder to override
- CSS modules: More boilerplate for each component
- Styled-components: Runtime overhead, not needed for static site

**Configuration**:
```js
// docusaurus.config.js
plugins: [
  'docusaurus-tailwindcss',
],
```

### 4. GitHub Pages Deployment

**Decision**: Use GitHub Actions for automated deployment on push to main

**Rationale**:
- Free hosting for public repos
- Fast CDN delivery globally
- Integrated with GitHub workflow
- Custom domain support if needed

**Alternatives Considered**:
- Vercel: Great but overkill for static site
- Netlify: Good but GitHub Pages is more familiar
- Self-hosted: Unnecessary complexity

**GitHub Actions Workflow**:
```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages
on:
  push:
    branches: [main]
jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
      - run: npm ci
        working-directory: frontend
      - run: npm run build
        working-directory: frontend
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./frontend/build
```

### 5. Chapter Content Structure

**Decision**: Use folder-based chapters with `_category_.json` for organization

**Rationale**:
- Docusaurus convention for multi-file chapters
- Automatic sidebar generation from folder structure
- Clear separation of chapter sections
- Easy to reorder with position property

**Alternatives Considered**:
- Single large file per chapter: Hard to edit, slow to load
- JSON-based sidebar: More manual work, harder to maintain

**Chapter Structure**:
```text
docs/
├── intro.md                    # position: 1
├── module-1-ros2/
│   ├── _category_.json         # { "label": "ROS2 Fundamentals", "position": 2 }
│   ├── index.md                # Chapter overview
│   ├── 01-nodes-topics.md      # Section 1
│   └── 02-services-actions.md  # Section 2
```

### 6. Mobile Responsiveness

**Decision**: Use Docusaurus default responsive behavior + custom breakpoints

**Rationale**:
- Docusaurus classic theme is already mobile-responsive
- Sidebar collapses to hamburger menu on mobile
- Code blocks get horizontal scroll
- Tables become scrollable

**Key CSS Breakpoints**:
```css
/* Custom CSS for edge cases */
@media (max-width: 768px) {
  .code-block-container { overflow-x: auto; }
}
@media (max-width: 375px) {
  /* iPhone SE minimum width support */
}
```

### 7. Code Syntax Highlighting

**Decision**: Use Prism with additional language support for robotics

**Rationale**:
- Docusaurus uses Prism by default
- Support for Python, C++, YAML, Bash needed for robotics content
- Copy button built-in
- Line highlighting for tutorials

**Configuration**:
```js
// docusaurus.config.js
prism: {
  theme: prismThemes.github,
  darkTheme: prismThemes.dracula,
  additionalLanguages: ['python', 'cpp', 'yaml', 'bash', 'json'],
},
```

### 8. Image Optimization

**Decision**: Use static/img folder with manual optimization + lazy loading

**Rationale**:
- Docusaurus handles image imports well
- Static folder is copied directly to build
- Lazy loading improves initial page load
- No build-time processing needed

**Alternatives Considered**:
- @docusaurus/plugin-ideal-image: More complex setup
- External CDN: Adds dependency

**Best Practices**:
- Compress images to <100KB each
- Use WebP format where supported
- Add descriptive alt text for accessibility
- Use responsive images for diagrams

## Technology Decisions Summary

| Decision | Choice | Confidence |
|----------|--------|------------|
| Framework | Docusaurus 3.x + TypeScript | High |
| Search | @easyops-cn/docusaurus-search-local | High |
| Styling | Tailwind CSS via plugin | High |
| Hosting | GitHub Pages + Actions | High |
| Content Structure | Folder-based chapters | High |
| Code Highlighting | Prism (default) | High |
| Images | Static folder + lazy loading | Medium |

## Risks Identified

1. **Content Volume Risk**: 4 chapters × 2000 words = 8000 words minimum. Tight deadline.
   - Mitigation: Start content writing immediately after scaffolding

2. **Search Index Size**: Large content may slow search
   - Mitigation: Monitor build size, consider Algolia if needed

3. **Image Loading**: Many diagrams could slow page load
   - Mitigation: Compress all images, use lazy loading

## Next Steps

1. Scaffold Docusaurus project with TypeScript
2. Configure search and Tailwind plugins
3. Set up GitHub Actions deployment
4. Create chapter folder structure
5. Begin content writing (Module 1: ROS2)

