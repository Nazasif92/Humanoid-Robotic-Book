# ADR-001: Book Platform and Publishing Architecture

**Status**: Proposed
**Date**: 2025-12-05
**Deciders**: Architecture Team
**Affected Components**: Documentation site, CI/CD, hosting, reader access

## Context

The Humanoid Robotics Book requires a publishing platform that:
- Maintains source control (version history, collaborative editing)
- Automatically builds and deploys to public internet
- Supports technical content (code blocks, diagrams, callouts, math notation)
- Provides excellent search and navigation for multi-chapter book
- Requires minimal DevOps overhead for maintenance
- Must be beginner-friendly for content creators (Markdown-based)

Key constraints:
- Book is educational, not commercial—no need for e-commerce, subscriptions, or analytics infrastructure
- Target audience: beginners with varying technical backgrounds (WSL2, Docker, native Linux)
- Content will change frequently (new chapters, code example updates, clarifications)
- All content should be version-controlled and open-source ready

## Decision

**Chosen Approach: Docusaurus v3+ on GitHub Pages with GitHub Actions CI/CD**

### Components:
1. **Static Site Generator**: Docusaurus v3+ (React-based, MDX support)
2. **Hosting**: GitHub Pages (free tier)
3. **Source Control**: Git repository on GitHub
4. **Build Automation**: GitHub Actions workflow (auto-build on push to main)
5. **Content Format**: Markdown (.md) with MDX extensions for interactive content
6. **Documentation Structure**: Docs-only site (no marketing landing page, focused on content)
7. **Theme**: Classic theme (light mode default, dark mode toggle available)
8. **Deployment Branch**: `gh-pages` (auto-managed by Actions)

### Architecture Rationale:
- **Docusaurus**: Purpose-built for technical documentation; integrates well with code snippets, diagrams (Mermaid), and callout boxes
- **GitHub Pages**: Free, automatic HTTPS, integrated with version control, no ops overhead
- **GitHub Actions**: Eliminates manual deploy steps; one-command publish (git push)
- **Markdown + MDX**: Version-controllable, human-readable, widely supported
- **Docs-only**: Keeps focus on content; readers navigate directly to chapters; no homepage bloat

## Consequences

### Positive:
- **Low overhead**: No servers to manage, no hosting bills, no DevOps complexity
- **Fast iteration**: Change → commit → auto-deploy within minutes
- **Team collaboration**: All contributors use Git (familiar workflow)
- **Open-source ready**: Reader can fork, submit PRs, contribute fixes
- **SEO-friendly**: GitHub Pages, Docusaurus auto-generate sitemaps and meta tags
- **Offline reading**: Generated static site can be downloaded and read locally

### Negative:
- **GitHub dependency**: If GitHub experiences outages, site goes down; limited customization (GitHub Pages static hosting only)
- **No server-side logic**: Can't dynamically generate content, track readers, or collect usage metrics
- **Monolithic build**: Large books may have longer build times (mitigated by caching)
- **Learning curve**: Team must understand GitHub Actions YAML syntax for CI/CD modifications
- **Styling limits**: Theme customization requires forking Docusaurus or custom CSS (within constraints)

### Risks:
- **Build failures**: If dependencies update and break (mitigated by pinning package versions)
- **Large file size**: Many chapters + diagrams → larger build artifact (mitigated by image optimization)
- **Domain management**: Custom domain requires DNS configuration (future concern, not blocking)

## Alternatives Considered

### Alternative 1: Traditional CMS (WordPress + Hosting)
- **Approach**: Self-hosted WordPress or Wix/Squarespace
- **Pros**: WYSIWYG editor, built-in user management, analytics, plugins
- **Cons**: Monthly hosting fees ($10–100/month), DevOps maintenance, overkill for static content, harder to version-control
- **Rejected because**: Too much infrastructure overhead; content is static; Git versioning is superior for collaborative writing

### Alternative 2: Jupyter Book + ReadTheDocs
- **Approach**: Jupyter Book format (.ipynb files) deployed on ReadTheDocs
- **Pros**: Supports interactive code cells, mathematical notation native
- **Cons**: Heavier for pure content (code cells are optional here), slightly steeper learning curve for writers
- **Rejected because**: Our book is narrative-first, not notebook-first; Docusaurus is lighter and faster

### Alternative 3: GitBook
- **Approach**: GitBook platform (free tier, Git integration)
- **Pros**: Beautiful UI, Git sync built-in, WYSIWYG editor available
- **Cons**: Limited customization on free tier, vendor lock-in (closed platform), paid tiers needed for advanced features
- **Rejected because**: GitHub Pages is more open and has no per-tier limitations

### Alternative 4: Sphinx + ReadTheDocs
- **Approach**: Sphinx (Python documentation tool) + ReadTheDocs hosting
- **Pros**: Industry standard for Python docs, powerful customization
- **Cons**: Steeper learning curve (reStructuredText syntax), overkill for this project's scope
- **Rejected because**: Docusaurus is more intuitive for non-Python audiences; MDX support is superior

## Implementation Plan

1. **Phase 1 (Weeks 1–2)**:
   - Initialize Docusaurus project locally (`npx create-docusaurus@latest`)
   - Configure `docusaurus.config.js` (title, URL, GitHub org, baseUrl)
   - Set up GitHub repository with `docs/` structure
   - Create `.github/workflows/deploy.yml` for GitHub Actions
   - Test local build: `npm run build` → manual inspection → success

2. **Phase 2 (Weeks 2–3)**:
   - Write Chapters 1–2
   - Test rendering in dev server (`npm run start`)
   - Commit and push; verify GitHub Actions trigger and deploy

3. **Phase 3 (Weeks 3–4)**:
   - Write Chapters 3–4
   - Verify all links (internal and external)

4. **Phase 4 (Weeks 4–5)**:
   - Write appendices
   - Final build and deploy to `gh-pages`
   - Enable GitHub Pages in repository settings
   - Verify site is live and accessible

## Validation Criteria

- ✅ Local build succeeds (`npm run build` produces `./build/` directory)
- ✅ GitHub Pages deployment successful (site accessible at `https://[user].github.io/humanoid-robotic-book/`)
- ✅ All chapters render with correct formatting, code blocks, and diagrams
- ✅ Navigation sidebar works (sidebar links, next/previous chapter links)
- ✅ Search functionality works (Docusaurus default search)
- ✅ Mobile rendering is responsive and readable

## References

- **Plan**: `specs/001-ros2-chapter/plan.md` (PART 4: Docusaurus Integration & Deployment)
- **Constitution**: `.specify/memory/constitution.md` (Publication requirement: "Must build and deploy cleanly to GitHub Pages")
- **Docusaurus Docs**: https://docusaurus.io/docs/deployment
- **GitHub Actions Docs**: https://docs.github.com/en/actions/quickstart
- **GitHub Pages Docs**: https://docs.github.com/en/pages
