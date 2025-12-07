# ADR-005: Docusaurus Theme and Layout Architecture

**Status**: Proposed
**Date**: 2025-12-05
**Feature**: 001-ros2-chapter (spans all 4 modules)
**Context**: The book requires a publishing platform that presents 4 chapters + 4 appendices with clear navigation, search, and accessibility. Key decisions:

- Should the site be docs-only (focused content) or full website (marketing + content)?
- What theme and color mode (light-first or dark-first)?
- How to structure navigation (sidebar, topbar, footer)?
- What components and callout types support pedagogical goals?

## Decision

**Chosen Approach: Docs-Only Site with Classic Theme (Light Default, Dark Toggle), Left Sidebar Navigation**

### Components:

**1. Site Architecture: Docs-Only (Not Full Website)**
- **Purpose**: Focus reader attention on chapter content; no homepage, marketing, or landing page bloat
- **Structure**: Docusaurus docs plugin (not full website theme)
- **Homepage**: Minimal intro page that redirects to Chapter 1
- **Navigation**: Sidebar (chapters + appendices) + top search bar + footer (GitHub link, license)
- **Rationale**: Educational content benefits from stripped-down UI; readers navigate sequentially; no distraction

**2. Theme: Classic with Light Mode Default**
- **Theme Plugin**: Docusaurus Classic theme (built-in, stable, no dependencies)
- **Color Scheme**:
  - Default: Light mode (white background, dark text)
  - Toggle: Dark mode available (gray background, light text)
- **Rationale**: Light mode is beginner-friendly (easier on eyes during long reading), dark mode available for preference
- **Customization**: Light mode: #ffffff bg, #1a1a1a text; Dark mode: #1a1a1a bg, #ffffff text

**3. Navigation Layout**

```
┌─────────────────────────────────────────────────────┐
│ Humanoid Robotics Book  [Search] [🌙 Dark Mode]     │  ← Top bar
├────────────────────────────────────────────────────┤
│ Sidebar               │  Main Content     │ TOC      │
│                       │                   │          │
│ Introduction          │ Chapter 2:        │ Contents │
│ ├─ Chapter 1          │ Digital Twin      │ - 2.1    │
│ ├─ Chapter 2          │                   │ - 2.2    │
│ ├─ Chapter 3          │ Lorem ipsum dolor │ - 2.3    │
│ ├─ Chapter 4          │                   │ - 2.4    │
│ └─ Appendices         │ [Diagram 1]       │          │
│   ├─ Setup            │                   │ Top ↑    │
│   ├─ Examples         │ [Prev] [Next] →   │          │
│   ├─ Glossary         │                   │          │
│   └─ Further Reading  │                   │          │
│                       │                   │          │
└────────────────────────────────────────────────────┘
         (On mobile: sidebar collapses)
```

**Components**:
- **Left Sidebar**: Chapters and appendices in hierarchical tree; active chapter highlighted
- **Top Search Bar**: Docusaurus-native search (Algolia or local search)
- **Right TOC (Table of Contents)**: Headings in current page (h2, h3); sticky on desktop
- **Previous/Next Navigation**: At chapter bottom (e.g., "[← Chapter 1]  [Chapter 3 →]")
- **Footer**: Minimal (copyright, GitHub link, license)
- **Mobile**: Sidebar collapses into hamburger menu; TOC moves to bottom

**4. Content Components & Callout Styles**

Docusaurus supports these callout types via MDX:

```markdown
:::note
This is a note callout.
:::

:::info
This provides additional information.
:::

:::warning
This is an important warning.
:::

:::danger
This is a critical warning (e.g., safety).
:::

:::tip
This is a helpful tip or best practice.
:::
```

**Usage Strategy by Module**:

| Callout Type | Usage | Example |
|---|---|---|
| `:::note` | Definition or clarification | "A ROS 2 node is a single executable process..." |
| `:::info` | Additional context or background | "This concept applies to real robots too" |
| `:::tip` | Best practices, hints, shortcuts | "Use `ros2 topic echo` to debug topics" |
| `:::warning` | Prerequisites, cautions, common mistakes | "Don't skip Chapter 1; you need ROS 2 concepts" |
| `:::danger` | Safety, critical errors, security issues | "Never run untested code on a real robot" |

**5. Sidebar Configuration (sidebars.js)**

```javascript
module.exports = {
  docsSidebar: [
    'intro',  // Introduction page
    {
      label: 'Chapters',
      items: [
        'chapters/01-ros2-nervous-system',
        'chapters/02-digital-twin-simulation',
        'chapters/03-isaac-perception',
        'chapters/04-vla-humanoid',
      ],
    },
    {
      label: 'Appendices',
      items: [
        'appendices/00-environment-setup',
        'appendices/01-code-examples',
        'appendices/02-glossary',
        'appendices/03-further-reading',
      ],
    },
  ],
};
```

**6. docusaurus.config.js Theme Configuration**

```javascript
module.exports = {
  title: 'Humanoid Robotics Book',
  tagline: 'Learn ROS 2, Digital Twins, AI Perception, and Vision-Language Action',
  url: 'https://[username].github.io',
  baseUrl: '/humanoid-robotic-book/',
  deploymentBranch: 'gh-pages',
  themeConfig: {
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,  // Allow dark mode toggle
      respectPrefersColorScheme: true,  // Respect OS preference
    },
    navbar: {
      title: 'Humanoid Robotics Book',
      logo: { alt: 'Logo', src: 'img/logo.svg' },
      items: [
        { to: 'docs/intro', label: 'Read', position: 'right' },
        { href: 'https://github.com/[repo]', label: 'GitHub', position: 'right' },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [ { label: 'Chapter 1', to: 'docs/chapters/01-ros2-nervous-system' } ],
        },
        {
          title: 'Community',
          items: [ { label: 'GitHub Issues', href: 'https://github.com/[repo]/issues' } ],
        },
      ],
      copyright: `© 2025 Humanoid Robotics Book. Licensed under CC BY-NC 4.0.`,
    },
  },
};
```

**7. Accessibility Features**

- **Keyboard Navigation**: Tab through sidebar, search bar, content; Enter to activate links
- **Color Contrast**: Light theme meets WCAG AA standard (white bg, dark text); dark theme tested
- **Screen Reader**: Semantic HTML, proper heading hierarchy, alt text for diagrams
- **Search Accessibility**: Native Docusaurus search is keyboard accessible
- **Focus Indicators**: Visual outline on focused elements

## Consequences

### Positive

- **Laser-focused content**: No marketing fluff; readers land on Chapter 1 immediately
- **Familiar UX**: Docusaurus Classic theme is widely used for technical docs; readers expect this layout
- **Fast performance**: Docs-only site is lightweight; pages load quickly
- **Easy maintenance**: Classic theme is stable, well-maintained, minimal customization needed
- **Accessibility by default**: Docusaurus ships with good accessibility; light-first + dark toggle is best practice
- **Mobile-friendly**: Sidebar collapses gracefully; responsive out-of-the-box
- **Search effectiveness**: Default search works well for 4–6 chapters; no need for Algolia setup
- **Beginner-friendly**: No cognitive load from navigation; sidebar is intuitive tree structure

### Negative

- **Limited visual branding**: Docs-only site may feel minimal (no hero image, no marketing); mitigated by focusing on content quality
- **No built-in analytics**: Can't track reader behavior (mitigated by constitution: no analytics required)
- **Dark mode trade-off**: Light-first may not appeal to all readers; dark mode is secondary (mitigated by toggle)
- **Customization ceiling**: Classic theme limits design flexibility; major changes require theme forking (mitigated by constitution: simplicity over customization)
- **No e-commerce or user accounts**: If future version needs subscriptions, platform must change (mitigated by current scope: educational, free)

### Risks

- **Light mode readability**: Some readers prefer dark mode by default (mitigated by OS color scheme detection: `respectPrefersColorScheme: true`)
- **Sidebar clutter**: If appendices grow beyond 4, sidebar becomes unwieldy (mitigated by collapsible sections in sidebars.js)
- **Search limitations**: If book grows to 10+ chapters, default search may need Algolia (mitigated by pinning to 4 chapters in constitution)

## Alternatives Considered

### Alternative 1: Full Marketing Website (Homepage + Docs)
- **Approach**: Add hero landing page, feature highlights, author bios, testimonials; docs nested under `/docs/`
- **Pros**: Professional appearance, marketing appeal, SEO-friendly (more content for search engines)
- **Cons**: Scope creep (adds landing page design, copy, screenshots); distraction from core content; maintenance burden; beginners may get lost navigating marketing + docs
- **Rejected because**: Book is educational, not commercial; focus should be on content, not marketing; constitution prioritizes simplicity

### Alternative 2: Dark Mode First (Dark Default, Light Toggle)
- **Approach**: Reverse color scheme; dark bg by default
- **Pros**: Modern feel, appeals to developers (dark mode convention), less eye strain for some
- **Cons**: Less suitable for beginners (technical tutorials traditionally use light mode); accessibility trade-off (light text on dark needs higher contrast); dark theme may reduce readability for older readers
- **Rejected because**: Light mode is pedagogically better for beginners; dark toggle available for preference

### Alternative 3: Custom Theme (Not Classic)
- **Approach**: Develop custom Docusaurus theme using swizzling and custom components
- **Pros**: Full design control, unique branding, custom components (e.g., interactive quizzes)
- **Cons**: Maintenance overhead, steep learning curve, breaks with future Docusaurus updates, no out-of-the-box accessibility
- **Rejected because**: Violates constitution principle (simplicity); custom theme is scope creep; Classic theme meets all requirements

### Alternative 4: Blog + Docs Hybrid
- **Approach**: Mix Docusaurus docs plugin (chapters) with blog plugin (updates, news)
- **Pros**: Can announce new chapters or corrections
- **Cons**: Adds complexity; readers may confuse blog posts with chapter content; blog maintenance overhead
- **Rejected because**: Book is static; no need for blog; if updates needed, use GitHub releases or simple README changelog

### Alternative 5: Multiple Themes (Responsive Layout)
- **Approach**: Use different layout on desktop (3-column: sidebar + content + TOC) vs. mobile (full-width)
- **Pros**: Optimal for each device size
- **Cons**: Complex CSS, potential inconsistencies, harder to maintain
- **Rejected because**: Docusaurus handles responsive layout automatically; we can trust the built-in responsiveness

## Implementation Plan

1. **Phase 1 (Week 1): Setup & Configuration**
   - Clone Docusaurus starter project
   - Configure `docusaurus.config.js` with title, URL, color mode settings
   - Configure `sidebars.js` with chapter + appendix structure
   - Create placeholder intro page

2. **Phase 2 (Week 1–2): Theme Customization**
   - Set colors (light: #ffffff bg, #1a1a1a text; dark: #1a1a1a bg, #ffffff text)
   - Add project logo (if desired)
   - Customize footer (copyright, GitHub link)
   - Test light + dark mode switching

3. **Phase 3 (Week 2): Component & Callout Styles**
   - Create MDX callout examples (note, info, tip, warning, danger)
   - Test rendering in dev server
   - Document usage in contributor guide

4. **Phase 4 (Week 2–3): Navigation & Search**
   - Test sidebar navigation on desktop + mobile
   - Verify search functionality (default search or setup Algolia if needed)
   - Test previous/next chapter buttons

5. **Phase 5 (Week 3): Accessibility Audit**
   - Run WAVE or axe DevTools
   - Verify keyboard navigation
   - Test screen reader (NVDA/JAWS for Windows; VoiceOver for Mac)
   - Verify color contrast ratios

6. **Phase 6 (Week 3–4): Integration with Chapters**
   - Place Chapter 1–4 Markdown files in `docs/chapters/`
   - Place Appendix files in `docs/appendices/`
   - Verify sidebar links to all chapters
   - Test full build: `npm run build`

7. **Phase 7 (Week 4–5): Polish & Deployment**
   - Final review of layout, colors, navigation
   - GitHub Actions deployment test
   - Live GitHub Pages deployment
   - Verify all links (internal + external)
   - Final accessibility pass

## Validation Criteria

- ✅ Light mode renders correctly (colors, contrast, readability)
- ✅ Dark mode renders correctly (toggle works, colors readable)
- ✅ Sidebar navigation shows all 4 chapters + 4 appendices hierarchically
- ✅ Search bar is functional and returns relevant results
- ✅ Previous/next chapter buttons work on all chapter pages
- ✅ Mobile layout: sidebar collapses into hamburger menu; content readable on small screens
- ✅ Keyboard navigation: Tab through all interactive elements; Enter activates links
- ✅ Screen reader: Page structure, headings, alt text all readable
- ✅ WCAG AA contrast ratio met for both light and dark modes
- ✅ Docusaurus build completes without errors
- ✅ GitHub Pages deployment successful; site live and accessible

## References

- **Plan**: `specs/001-ros2-chapter/plan.md` (PART 6.5: Docusaurus Theme, PART 4: Docusaurus Integration)
- **ADR-001**: Book Platform and Publishing Architecture (Docusaurus + GitHub Pages decision)
- **Constitution**: `.specify/memory/constitution.md` (Publication requirement, simplicity principle)
- **Docusaurus Docs**: https://docusaurus.io/docs
- **Docusaurus Theme Configuration**: https://docusaurus.io/docs/next/styling-layout
- **Docusaurus Callouts/Admonitions**: https://docusaurus.io/docs/markdown-features/admonitions
- **WCAG Accessibility**: https://www.w3.org/WAI/WCAG21/quickref/
