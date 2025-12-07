# ADR-006: Quality Validation and Review Process

**Status**: Proposed
**Date**: 2025-12-05
**Feature**: 001-ros2-chapter (spans all 4 modules)
**Context**: The book aims to serve beginners with accurate, clear technical content. Each chapter must meet quality standards before publication. Key decisions:

- How many review gates before a chapter is accepted?
- What criteria define "acceptable quality"?
- Who reviews (technical experts, instructional designers, community)?
- How to catch errors early (fact-checking, code validation, accessibility)?

## Decision

**Chosen Approach: 5-Gate Validation Process with Parallel Review Workflow**

### Gate Structure:

```
Chapter Draft
    ↓
Gate 1: Functional Requirements Validation
    ↓ (PASS/FAIL)
Gate 2: Technical Accuracy Review
    ↓ (PASS/FAIL)
Gate 3: Comprehension & Clarity Review
    ↓ (PASS/FAIL)
Gate 4: Consistency & Alignment Review
    ↓ (PASS/FAIL)
Gate 5: Build & Deployment Verification
    ↓ (PASS/FAIL)
    ↓
Published Chapter
```

### Gate Definitions:

**Gate 1: Functional Requirements Validation**
- **Purpose**: Verify chapter addresses all FRs from corresponding spec
- **Checkpoints**:
  - ✅ All learning objectives covered (from spec)
  - ✅ All user stories addressed
  - ✅ Required diagrams present (title + caption)
  - ✅ Code examples (Ch 1–2) or pseudocode (Ch 3–4) complete
  - ✅ All technical terms introduced and explained
  - ✅ Chapter word count: 800–1200 words (excluding code blocks)
  - ✅ Reference citations present and formatted (APA 7)
  - ✅ Chapter follows structure: intro → sections → summary → next
- **Approver**: Content lead or product owner
- **Tool**: Checklist in Markdown (docs/VALIDATION_GATES/gate1-fr-validation.md)
- **Time**: ~30 minutes per chapter

**Gate 2: Technical Accuracy Review**
- **Purpose**: Verify code examples run, concepts are correct, terminology is precise
- **Checkpoints**:
  - ✅ Code examples (Ch 1–2) tested in target environment (ROS 2 Humble, Gazebo, Docker)
  - ✅ URDF/SDF files validate (correct XML, no syntax errors)
  - ✅ Launch files execute without missing dependencies
  - ✅ Pseudocode (Ch 3–4) follows ROS 2 conventions and is idiomatically correct
  - ✅ Technical terms aligned with official documentation (ROS 2 docs, Isaac docs, etc.)
  - ✅ Diagrams are architecturally correct (no misrepresentations of systems)
  - ✅ Facts verified against primary sources (links to official docs)
  - ✅ No deprecated APIs or outdated information
- **Approver**: Technical expert (ROS 2 maintainer, Isaac expert, ML engineer)
- **Tool**: Test harness + fact-check matrix (docs/VALIDATION_GATES/gate2-accuracy-review.md)
- **Time**: ~1–2 hours per chapter (includes running code)

**Gate 3: Comprehension & Clarity Review**
- **Purpose**: Verify beginners can understand content; writing is clear and jargon is minimized
- **Checkpoints**:
  - ✅ No unexplained jargon (all technical terms defined in context or in glossary)
  - ✅ Explanations are step-by-step (not leaping between concepts)
  - ✅ Examples are relatable and motivating for beginners
  - ✅ Sentence structure is clear (no overly complex grammar)
  - ✅ Callout boxes (:::note, :::tip, :::warning) highlight key concepts
  - ✅ Diagrams have captions and are referenced in text
  - ✅ Chapter summary recaps learning objectives
  - ✅ "Next chapter" teaser motivates continued reading
- **Approver**: Instructional designer or experienced educator
- **Tool**: Readability checklist + peer review feedback (docs/VALIDATION_GATES/gate3-clarity-review.md)
- **Time**: ~45 minutes per chapter (reading + feedback)

**Gate 4: Consistency & Alignment Review**
- **Purpose**: Verify chapter aligns with prior chapters, uses consistent terminology, and maintains style
- **Checkpoints**:
  - ✅ Terminology consistent with earlier chapters (no "node" vs. "process" confusion)
  - ✅ References to prior chapters are accurate (e.g., "Recall from Chapter 1...")
  - ✅ Markdown style consistent (heading levels, code block formatting, callout types)
  - ✅ Diagram styles match prior chapters (colors, fonts, notation)
  - ✅ Code style consistent (naming conventions, comments, indentation match Chapter 1–2 examples)
  - ✅ Citation format consistent (all APA 7 format)
  - ✅ Chapter prerequisites state what prior knowledge is needed
  - ✅ No broken internal links (links to prior chapters work)
- **Approver**: Content lead or curriculum designer
- **Tool**: Cross-chapter alignment matrix (docs/VALIDATION_GATES/gate4-consistency-review.md)
- **Time**: ~1 hour per chapter (requires reviewing prior chapters)

**Gate 5: Build & Deployment Verification**
- **Purpose**: Verify Docusaurus build succeeds, all links render, site deployable, accessibility passes
- **Checkpoints**:
  - ✅ `npm run build` completes without errors or warnings
  - ✅ Chapter renders correctly in Docusaurus dev server (`npm run start`)
  - ✅ Sidebar link to chapter works
  - ✅ Previous/next chapter buttons work
  - ✅ All internal links (e.g., links to other chapters, appendices) are valid
  - ✅ All external links are valid (HTTP 200 or 301, not 404)
  - ✅ Images load correctly; all images have alt text
  - ✅ Code blocks syntax-highlight correctly (language tag matches content)
  - ✅ Mermaid diagrams render without errors
  - ✅ Mobile layout: readable on phone/tablet
  - ✅ Dark mode: chapter readable with dark theme enabled
  - ✅ Accessibility test passes (WAVE, axe DevTools, or manual screen reader check)
  - ✅ GitHub Actions deployment workflow succeeds
  - ✅ Chapter appears on live GitHub Pages site
- **Approver**: DevOps/build engineer or technical lead
- **Tool**: Automated CI/CD checks + manual browser test (docs/VALIDATION_GATES/gate5-build-deployment.md)
- **Time**: ~20 minutes per chapter (mostly automated)

### Review Workflow (Parallel Execution):

```
Gate 1 (FR Validation) — Content Lead — 30 min
    ↓ (PASS) ↑ (FAIL → revise → retry)

    ├→ Gate 2 (Accuracy) — Tech Expert — 1–2 hours (parallel)
    ├→ Gate 3 (Clarity) — Educator — 45 min (parallel)
    └→ Gate 4 (Consistency) — Curriculum Designer — 1 hour (parallel)

    ↓ (all PASS)

Gate 5 (Build/Deploy) — DevOps/Tech Lead — 20 min
    ↓ (PASS)

✅ Chapter Published
```

**Parallel Execution**: Gates 2, 3, 4 can run in parallel after Gate 1 passes. Gate 5 runs after all pass. This reduces total review time from ~4.5 hours (sequential) to ~2.5 hours (parallel).

### Tools & Artifacts:

**Validation Checklists** (stored in `docs/VALIDATION_GATES/`):
- `gate1-fr-validation.md` — Checkbox list matching chapter to spec FRs
- `gate2-accuracy-review.md` — Code test results, fact-checking matrix, primary sources
- `gate3-clarity-review.md` — Readability feedback, peer review notes
- `gate4-consistency-review.md` — Cross-chapter terminology audit, alignment matrix
- `gate5-build-deployment.md` — Build log, link validation report, accessibility report

**GitHub Workflow**:
- Chapter draft submitted as pull request (PR)
- Each gate triggers GitHub Actions or manual review (via PR comments)
- Status checks block merge until all gates pass
- PR template includes gate checklist:
  ```markdown
  - [ ] Gate 1: FR Validation (Content Lead)
  - [ ] Gate 2: Technical Accuracy (Tech Expert)
  - [ ] Gate 3: Comprehension & Clarity (Educator)
  - [ ] Gate 4: Consistency & Alignment (Curriculum Designer)
  - [ ] Gate 5: Build & Deployment (DevOps)
  ```

**FAIL → Revise → Retry Cycle**:
- If chapter fails any gate: author revises based on feedback
- Specific issues documented in PR comments
- Author re-submits; reviewers re-check same gate
- Process repeats until PASS

### Success Criteria per Chapter:

Each chapter must achieve:
- ✅ 100% FR coverage (no missing learning objectives)
- ✅ 0 code errors (all examples in Ch 1–2 run; all pseudocode in Ch 3–4 is correct)
- ✅ <5 readability issues (jargon explained, clarity high)
- ✅ 100% terminology consistency with prior chapters
- ✅ 0 broken links; all images render
- ✅ WCAG AA accessibility compliance
- ✅ Docusaurus build passes; GitHub Pages deployment successful

## Consequences

### Positive

- **Quality assurance**: 5 gates catch different types of errors (requirements → accuracy → clarity → consistency → deployment)
- **Parallel execution**: Gates 2–4 run concurrently, reducing total review time
- **Clear acceptance criteria**: Chapter authors know exactly what's expected
- **Beginner focus**: Gate 3 (clarity) ensures content is truly beginner-friendly
- **Reduced bugs**: Testing code early (Gate 2) prevents broken examples in published book
- **Accessibility guaranteed**: Gate 5 includes accessibility audit; no exclusion of readers
- **Consistency maintained**: Gate 4 ensures chapters form cohesive narrative
- **Traceability**: All decisions documented in checklists and PR comments; easy to trace why chapter was rejected

### Negative

- **Review overhead**: Each chapter requires ~2.5 hours review time (5 people × multiple hours)
- **Bottleneck potential**: If reviewers are unavailable, chapters pile up; may delay publication
- **Strictness may stifle creativity**: Authors may feel over-constrained by 5 gates; may reduce flexibility
- **Maintenance burden**: Checklists must be updated if process changes; templates may drift out of sync
- **False negatives possible**: Reviewer oversight could miss an error that slips through (mitigated by pair reviews on critical gates)

### Risks

- **Reviewer disagreement**: Tech expert says "accurate" but educator says "unclear"; conflict resolution needed (mitigated by clear escalation path: escalate to project lead)
- **Gate fatigue**: After 5 gates, authors may lose motivation (mitigated by celebrating chapters that pass all gates publicly)
- **Process becomes rigid**: Over time, gates may accumulate additional criteria, making process brittle (mitigated by constitution: regular review and simplification)

## Alternatives Considered

### Alternative 1: Single Gate (Fast-Track Approval)
- **Approach**: One reviewer checks all criteria (FR + accuracy + clarity + consistency + build)
- **Pros**: Fast, simple, one decision maker, reduced overhead
- **Cons**: Single reviewer may miss errors; no specialization (tech expert not ideal for clarity review); high cognitive load; likely quality suffers
- **Rejected because**: Educational content requires multiple perspectives; single reviewer is insufficient for quality assurance

### Alternative 2: Seven-Gate Process (More Gates)
- **Approach**: Add gate for "Prior Chapter Cohesion" and "Real-Robot Testing" (Ch 1–2)
- **Pros**: Even more thorough; catches edge cases
- **Cons**: Excessive overhead (~6+ hours per chapter); delays publication; violates constitution (simplicity); diminishing returns beyond 5 gates
- **Rejected because**: 5 gates already cover critical quality areas; additional gates are overhead

### Alternative 3: Community Review (Open-Source Model)
- **Approach**: Publish chapter drafts on GitHub; open for public comments (like Linux kernel review)
- **Pros**: Distributed expertise, catches errors author/reviewers missed, fosters community
- **Cons**: Chaotic feedback (hard to prioritize), no accountability, slow (could take weeks), off-topic comments, harsh tone potential
- **Rejected because**: Educational context is different from Linux kernel; students benefit from structured feedback, not open chaos

### Alternative 4: Automated Testing Only (No Human Review)
- **Approach**: Skip human gates; rely on GitHub Actions (linters, spellcheck, link validation, code testing)
- **Pros**: Fast, consistent, scalable, no human overhead
- **Cons**: Can't catch clarity issues (linter won't know if explanation is too advanced); can't verify pedagogical value; no human judgment; high false-negative rate
- **Rejected because**: Automation cannot replace human review for educational quality; code can run correctly but explanation can be confusing

### Alternative 5: Staggered Gates (Sequential, Not Parallel)
- **Approach**: Run all 5 gates sequentially, one after another
- **Pros**: Simple to manage, dependencies clear, no coordination needed
- **Cons**: Slow (all ~4.5 hours sequential vs. 2.5 parallel); wastes expert time (if Gate 1 fails, Gates 2–4 don't run, but we already scheduled reviewers); author waits between gates
- **Rejected because**: Parallel gates are more efficient; we can coordinate simultaneous reviews

## Implementation Plan

1. **Phase 1 (Week 1): Process Documentation**
   - Create gate checklist templates in `docs/VALIDATION_GATES/`
   - Document roles (content lead, tech expert, educator, curriculum designer, DevOps lead)
   - Create PR template with gate checklist
   - Define escalation path (if reviewers disagree, escalate to project lead)

2. **Phase 2 (Week 1–2): Reviewer Training**
   - Brief each reviewer role on their gate(s)
   - Share examples of PASS vs. FAIL
   - Establish turnaround time expectations (e.g., review within 2 days)

3. **Phase 3 (Week 2–3): Automated Checks (Gate 5 Support)**
   - Configure GitHub Actions workflow to run automated checks:
     - `npm run build` (Docusaurus build)
     - Link validator (check for 404s)
     - Spell checker (optional; can use VS Code extension)
     - Code syntax highlighter test
     - Accessibility checker (Axe or WAVE integration)
   - Store results as GitHub check status (block merge if failed)

4. **Phase 4 (Week 3–4): Chapter 1 Pilot Review**
   - Draft Chapter 1 (ROS 2)
   - Submit as PR; run all 5 gates
   - Document process, capture issues, refine checklists
   - Document time spent per gate
   - Publish Chapter 1 after passing all gates

5. **Phase 5 (Week 4–5): Scale to All Chapters**
   - Apply refined process to Chapters 2–4
   - Adjust parallel execution timing based on Chapter 1 experience
   - Publish chapters as they pass

6. **Phase 6 (Week 5): Process Refinement**
   - Retrospective: what worked, what slowed things down?
   - Update gate definitions and checklists based on learnings
   - Document final process

## Validation Criteria

- ✅ All 5 gate checklists created and available in `docs/VALIDATION_GATES/`
- ✅ GitHub PR template includes gate checklist; template enforced for all chapter PRs
- ✅ GitHub Actions workflow runs automated checks (build, links, code syntax); blocks merge on failure
- ✅ Each reviewer role is trained and documented
- ✅ Chapter 1 passes all 5 gates and is published
- ✅ Chapters 2–4 follow same 5-gate process
- ✅ 0 published chapters have broken examples (code runs without error)
- ✅ 0 published chapters have 404 links
- ✅ 0 published chapters fail accessibility audit (WCAG AA)
- ✅ Process runs in parallel (Gates 2–4 concurrent); total review time documented per chapter
- ✅ All gate decisions documented in PR comments (traceability)

## References

- **Plan**: `specs/001-ros2-chapter/plan.md` (PART 5: Quality Validation & Testing Strategy)
- **Constitution**: `.specify/memory/constitution.md` (Principle 5: Quality Review & Fact-Check)
- **All Module Specs**: Acceptance criteria, success criteria (validation gates must verify spec compliance)
- **ADR-001**: Book Platform and Publishing (GitHub Pages, which integrates with GitHub Actions for CI/CD)
- **GitHub Actions Docs**: https://docs.github.com/en/actions
- **Docusaurus Build**: https://docusaurus.io/docs/deployment
- **WCAG Accessibility**: https://www.w3.org/WAI/WCAG21/quickref/
- **APA 7 Citation Format**: https://apastyle.apa.org/
