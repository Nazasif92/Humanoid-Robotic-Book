<!--
SYNC IMPACT REPORT
==================
Version: 0.1.0 (initial)
Status: Created from user input
Date: 2025-12-05

Changes:
- Initial constitution created from project principles
- 5 core principles defined: Accuracy & Correctness, Beginner-Friendly, Consistency, Example-Driven, Quality Review
- Format & Technology Standards section added
- Content Constraints and Success Criteria sections included
- Governance rules established for constitution maintenance

Dependent Templates Status:
- ✅ plan-template.md: Compatible (Constitution Check section present)
- ✅ spec-template.md: Compatible (User scenarios focus aligns with principle)
- ✅ tasks-template.md: Compatible (Quality review requirement aligns with test-first emphasis)

Next Steps:
- Features created under this constitution should reference these principles in spec/plan/tasks
- Review process (mentioned in User Story 3) should validate principle adherence
- Consider ADR for "Docusaurus v3+ as single source of truth" if multi-platform publishing considered later

-->

# Humanoid Robotic Book Constitution

## Core Principles

### I. Accuracy & Correctness

All technical content must be verified against official documentation and tested before publication. Code samples must be correct, runnable, and produce expected output. Scientific claims must be cited to reliable sources. Errors discovered post-publication must be corrected immediately with clear versioning.

**Why**: The book's credibility depends on readers trusting that examples work and information is precise. Inaccuracies undermine learning and create technical debt in readers' code.

### II. Beginner-Friendly Language

Content must use simple, clear English without unnecessary jargon. Explanations follow step-by-step progression. Technical terms are defined on first use. Complex concepts are broken into smaller, digestible pieces with concrete examples before abstraction.

**Why**: Target audience is learners new to AI/robotics. Accessibility directly impacts learning outcomes and reader retention.

### III. Consistent Style & Structure

All chapters follow identical structure: introduction, learning objectives, step-by-step guide, practical examples, summary, key takeaways, and further reading. Code formatting, heading levels, and terminology are uniform across all chapters. Visual elements and code blocks use consistent styling.

**Why**: Consistency reduces cognitive load, helps readers navigate efficiently, and creates a professional presentation that signals reliability.

### IV. Example-Driven Explanations

Each concept is illustrated with working code samples and practical use cases. Examples progress from simple to complex. Readers can copy-paste examples and run them immediately without modification. When explaining features, show example output alongside code.

**Why**: People learn best by doing. Practical examples make abstract concepts concrete and allow immediate validation of understanding.

### V. Quality Review & Fact-Check

Every chapter must pass review gates before publication: technical accuracy review, beginner comprehension review, code execution test, and fact-check against official sources. Reviewers are external to the author when possible. Issues raised in review must be resolved or explicitly documented.

**Why**: Multiple reviewers catch errors individuals miss. External review ensures beginner-friendly language (authors become blind to assumed knowledge). Publishing only reviewed content maintains book quality.

## Format & Technology Standards

### Platform & Build

- **Platform**: Docusaurus v3+ with GitHub Pages
- **Format**: Markdown (`.md`) and MDX (`.mdx`) for interactive content
- **Build Tool**: Docusaurus CLI with `npm run build`
- **Deployment**: GitHub Actions workflow to GitHub Pages branch
- **Status**: Build must pass without errors before any deployment

### Content Structure

- **Repository**: Single GitHub repository (github.com/[org]/humanoid-robotic-book)
- **Documentation Root**: `/docs/` directory
- **Chapter File**: One `.md` file per chapter in `/docs/chapters/`
- **Assets**: Images/diagrams in `/docs/assets/`
- **Configuration**: `docusaurus.config.js` at repository root

### Styling & Formatting

- Use Markdown heading hierarchy: `#` for chapter, `##` for major section, `###` for subsection
- Code blocks with language specification (e.g., ````python`, ````javascript`)
- Link to official documentation using markdown links: `[Text](url)`
- Inline code with backticks: `` `code` ``
- No hardcoded styling; use Docusaurus components for interactive elements

## Content Constraints

### Chapter Scope

- **Target Length**: 800–1200 words per chapter for foundational modules; 1000–1500 words for complex integration modules
- **Chapters**: 8–12 chapters total (approximately 8,000–16,200 words)
- **Reading Time**: Each chapter readable in 8–15 minutes (based on complexity)
- **Scope**: Full coverage of [defined AI/robotics topic] from beginner to competent level
- **Complexity Guidelines**:
  - Modules 1 & 4 (Foundational & Capstone): 800–1200 words (core concepts)
  - Modules 2 & 3 (Integration & Advanced): 1000–1500 words (complex workflows requiring comprehensive coverage)

### Content Coverage

Each chapter must include:
1. Clear learning objectives (2–4 bullet points)
2. Step-by-step instructions or explanations
3. At least one working code example
4. Common mistakes or edge cases
5. Key takeaways (3–5 points)
6. Resources for deeper learning

### Code Quality

- All code examples must be tested and verified to run in a Docker container with Ubuntu 22.04 + ROS 2 Humble + Python 3.10
- Code must follow PEP 8 (Python), ESLint (JavaScript), or equivalent style guide
- Dependencies must be documented (e.g., "Requires Python 3.9+, numpy 1.20+")
- Error handling must be shown (try/catch, validation, etc.)
- Comments explain "why", not "what" (code is self-documenting)
- **Working Code Criteria**:
  - Zero runtime errors when executed in target environment
  - Deprecation warnings must be documented with mitigation plans
  - Expected output must be provided alongside all examples
  - Test commands with expected results must be included
  - All examples must be copy-paste runnable with no modifications required

## Success Criteria

### Build & Deployment

- ✅ **SC-001**: Docusaurus build completes without errors or warnings
- ✅ **SC-002**: GitHub Actions workflow deploys successfully to GitHub Pages
- ✅ **SC-003**: Published site is publicly accessible at [github.io URL]

### Content Quality

- ✅ **SC-004**: All 8–12 chapters are published
- ✅ **SC-005**: Every chapter passes technical accuracy review
- ✅ **SC-006**: Every chapter passes beginner comprehension review
- ✅ **SC-007**: Every code example is tested and produces expected output
- ✅ **SC-008**: Zero dead links; all references verified

### Reader Experience

- ✅ **SC-009**: Navigation is clear; chapters are easy to find and read
- ✅ **SC-010**: All code examples can be copy-pasted and run without modification
- ✅ **SC-011**: Readers report understanding core concepts after reading relevant chapters

### Governance

- ✅ **SC-012**: Completed book meets all Core Principles (Accuracy, Beginner-Friendly, Consistency, Example-Driven, Quality Review)

## Governance

### Constitution Authority

This constitution is the authoritative guide for book development. All decisions about content inclusion, style, technical approach, and quality gates must align with these principles. When ambiguity arises, principles take precedence over convenience.

### Amendment Process

1. **Proposed Amendment**: Document the proposed change and rationale
2. **Review**: Present to project lead and primary reviewers
3. **Validation**: Assess impact on existing chapters
4. **Approval**: Requires consensus; if conflict exists, principles take precedence
5. **Migration**: Update affected chapters or mark version bump if backward-incompatible
6. **Version Update**: Increment version per rules below

### Version Numbering

- **MAJOR**: Removal or redefinition of core principles (e.g., change from Docusaurus to another platform)
- **MINOR**: New principle added, or existing principle significantly expanded (e.g., add "Accessibility Standard")
- **PATCH**: Clarifications, wording improvements, typo fixes, non-semantic refinements

### Compliance & Review

- All PRs adding new chapters must reference this constitution
- Reviews must verify principle adherence (especially Accuracy, Example-Driven, Quality Review)
- Chapter authors must confirm: all code tested, sources cited, beginner-friendly language used, consistent style applied
- Publication blocked until all principles verified

### Escalation & Conflict Resolution

If reviewers disagree on principle interpretation, the project lead (as designated architect) makes the final decision, with rationale documented in the commit message or PR comment.

---

**Version**: 0.1.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
