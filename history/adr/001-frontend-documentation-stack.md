# ADR-001: Frontend Documentation Stack

> **Scope**: Frontend technology choices for the textbook website including framework, styling, search, and deployment.

- **Status:** Accepted
- **Date:** 2025-11-29
- **Feature:** 001-docusaurus-book
- **Context:** Need to build an AI-native technical textbook for Physical AI & Humanoid Robotics that supports rich content, search, theming, and can host embedded AI features (chatbot, podcast player).

## Decision

We will use the following frontend stack:

| Component | Choice | Version |
|-----------|--------|---------|
| **Framework** | Docusaurus | 3.x |
| **Styling** | Docusaurus default + Tailwind CSS | 3.x |
| **Search** | Docusaurus Local Search | Built-in |
| **Deployment** | GitHub Pages | N/A |
| **Theme** | Light/Dark toggle | Built-in |
| **Content Format** | MDX (Markdown + JSX) | Built-in |

### Key Configuration Decisions

- Use Docusaurus "docs" mode (not blog mode)
- Enable versioning for future curriculum updates
- Custom React components for: ChatBot widget, PodcastPlayer, DiagramViewer
- Static site generation for GitHub Pages compatibility

## Consequences

### Positive

- **Fast setup**: Docusaurus is specifically designed for documentation sites
- **Built-in features**: Search, versioning, i18n, dark mode all included
- **MDX support**: Can embed React components in markdown content
- **SEO optimized**: Static generation with proper meta tags
- **Free hosting**: GitHub Pages is free and reliable
- **Great DX**: Hot reload, TypeScript support, plugin ecosystem
- **Hackathon-friendly**: Can ship quickly with minimal configuration

### Negative

- **React dependency**: Must use React for custom components
- **Less flexibility**: Opinionated structure may limit customization
- **Bundle size**: Docusaurus adds overhead compared to plain HTML
- **GitHub Pages limits**: No server-side rendering, limited to static content

## Alternatives Considered

### Alternative A: Next.js + MDX + Vercel
- **Pros**: More flexible, SSR capability, better for dynamic features
- **Cons**: More setup required, overkill for documentation, slower to ship
- **Why rejected**: Hackathon timeline requires fast setup; Docusaurus is purpose-built for docs

### Alternative B: Astro + Starlight
- **Pros**: Faster builds, smaller bundles, island architecture
- **Cons**: Smaller ecosystem, less mature, fewer plugins
- **Why rejected**: Less familiar, fewer ready-made components for our use case

### Alternative C: GitBook / Notion
- **Pros**: Zero setup, WYSIWYG editing
- **Cons**: Limited customization, can't embed custom React components, vendor lock-in
- **Why rejected**: Cannot integrate RAG chatbot or custom AI features

### Alternative D: VitePress
- **Pros**: Vue-based, very fast, simple
- **Cons**: Vue ecosystem (team knows React better), fewer plugins
- **Why rejected**: Team expertise is in React

## References

- Feature Spec: `specs/001-docusaurus-book/spec.md`
- Implementation Plan: (pending)
- Related ADRs: ADR-002 (Backend API Stack), ADR-003 (AI Services Stack)
- Evaluator Evidence: N/A (decision based on hackathon constraints)
