# ADR-003: Learning Progression and Module Sequencing

**Status**: Proposed
**Date**: 2025-12-05
**Deciders**: Architecture Team
**Affected Components**: Chapter order, prerequisites, scaffolding, skill progression

## Context

The book covers four complex domains (ROS 2, simulation, AI/ML, language/vision) with strong interdependencies. Key decisions:

- What is the optimal reading order?
- Should readers be forced into a linear path or can they jump between chapters?
- How much prior knowledge must each chapter assume?
- How do we ensure earlier chapters prepare readers for later ones?

Initial tensions:
- Start with robotics fundamentals (ROS 2) or vision/language (more engaging)?
- Should AI chapters (3–4) require understanding simulation (Ch 2)?
- Can a reader skip Ch 2 and go straight to Ch 3?

## Decision

**Chosen Approach: Mandatory Linear Progression with Clear Prerequisites**

### Structure:
```
Chapter 1 (ROS 2)
  ↓ [required for Ch 2]
Chapter 2 (Digital Twin)
  ↓ [required for Ch 3]
Chapter 3 (Isaac + Perception)
  ↓ [required for Ch 4]
Chapter 4 (VLA — Capstone)
```

### Rationale:
1. **Ch 1 → Ch 2**: URDF (Ch 1 output) is directly used in Gazebo (Ch 2 input); ROS 2 nodes/topics (Ch 1) are critical for sensor data flow
2. **Ch 2 → Ch 3**: Digital twin simulation (Ch 2) provides foundation for understanding synthetic data and domain randomization (Ch 3)
3. **Ch 3 → Ch 4**: Isaac perception/planning stack (Ch 3) is the foundation for VLA's end-to-end workflow (Ch 4)

### Each Chapter States Prerequisites:
- **Ch 1**: "Basic Python knowledge; ROS 2 Humble installed (or Docker)"
- **Ch 2**: "Completed Ch 1; understanding of ROS 2 nodes and topics"
- **Ch 3**: "Completed Ch 1–2; familiarity with robot description (URDF)"
- **Ch 4**: "Completed Ch 1–3; understanding of ROS 2 ecosystem and simulation"

### Navigation Aids:
- Sidebar highlights current chapter and next recommended chapter
- Chapter introductions reference prior chapters ("Recall from Ch 1...")
- Each chapter ends with: "Next: Chapter X will..."

## Consequences

### Positive:
- **Scaffolding**: Each chapter builds on solid prior knowledge
- **Reduced cognitive load**: Beginners don't jump into AI without robotics context
- **Coherent narrative**: Readers see progression from basics → simulation → AI → autonomy
- **Reduced confusion**: "Do I need Chapter 2?" → Clear: Yes, it's prerequisite for Ch 3
- **Testing clarity**: Each chapter assumes specific prior knowledge; acceptance testing is precise

### Negative:
- **Less flexible**: Reader can't cherry-pick (e.g., "I only care about VLA")
- **Engagement risk**: Readers interested only in AI must wade through robotics chapters first (mitigated by summaries)
- **Skipping temptation**: Advanced readers may skip Ch 1–2; then get lost in Ch 3 (mitigated by clear prerequisite statements)

### Risks:
- **Attrition**: Readers drop out before reaching AI chapters (mitigated by beginner-friendly Ch 1–2)
- **Chapter cohesion**: If Ch 2 doesn't clearly connect to Ch 3, readers lose motivation (mitigated by explicit bridges)

## Alternatives Considered

### Alternative 1: Modular, Non-Linear
- **Approach**: Each chapter is independent; reader chooses order
- **Pros**: Maximum flexibility; appeals to different learner interests
- **Cons**: Repeated concepts; hard to maintain; readers get lost ("What's a node?" appears in Ch 1 and Ch 3)
- **Rejected because**: Too loose; scaffolding is lost; foundational concepts buried

### Alternative 2: Parallel Tracks (Beginner vs. Advanced)
- **Approach**: Two paths: "Beginner Track (Ch 1–2)" and "Advanced Track (Ch 3–4)"
- **Pros**: Flexibility; appeals to different levels
- **Cons**: Doubles maintenance; confusing navigation; makes book feel fragmented
- **Rejected because**: Our scope is already tight (4 chapters); this doubling is untenable

### Alternative 3: Spiral Progression (Layered Depth)
- **Approach**: Each topic appears in all chapters at increasing depth (e.g., ROS 2 in all 4 chapters)
- **Pros**: Readers revisit concepts; reinforces learning
- **Cons**: Very long book; risks repetition; hard to maintain consistency
- **Rejected because**: Would expand scope to 5000+ words; contradicts beginner focus

## Implementation Plan

1. **Sidebar Navigation**:
   - Visually highlight current chapter
   - Show "Next Chapter" button (links to Ch N+1)
   - Show reading time per chapter

2. **Chapter Introductions**:
   - Explicitly state prerequisites (link to prior chapters)
   - Include "Quick Review" section (1-2 paragraphs) of key concepts from prior chapter

3. **Chapter Endings**:
   - Summary of current chapter's outcomes
   - "What's Next" section (teaser for next chapter)
   - Link to next chapter

4. **Book Introduction** (landing page):
   - "This book is designed to be read sequentially. Each chapter builds on the previous one."
   - Clear visual progression diagram
   - Optional: "If you're familiar with [topic], you may skip to [chapter]"

5. **Appendix C (Glossary)**:
   - Alphabetical reference for all technical terms
   - Includes chapter/page reference for first usage

## Validation Criteria

- ✅ Each chapter's opening clearly states prerequisites
- ✅ Chapter-to-chapter transitions are smooth (no unexplained jumps)
- ✅ Sidebar navigation highlights current chapter and next chapter
- ✅ Chapter endings contain forward-looking teaser
- ✅ Reader can justify why Ch N is prerequisite for Ch N+1
- ✅ Glossary includes all technical terms with chapter references

## References

- **Plan**: `specs/001-ros2-chapter/plan.md` (PART 1.2: Learning Progression, PART 1.3: Module Interdependencies)
- **Module 1 Spec**: `specs/001-ros2-chapter/spec.md` (learning objectives: URDF output)
- **Module 2 Spec**: `specs/002-digital-twin/spec.md` (prerequisites: URDF input)
- **Module 3 Spec**: `specs/003-isaac-perception/spec.md` (assumptions: "Modules 1–2")
- **Module 4 Spec**: `specs/004-vla-humanoid/spec.md` (assumptions: "Modules 1–3")
