# ADR-002: Content Depth and Code Example Strategy

**Status**: Proposed
**Date**: 2025-12-05
**Deciders**: Architecture Team
**Affected Components**: Chapter content, code examples, complexity levels, reader prerequisites

## Context

The book spans four domains (ROS 2, simulation, AI/ML, language models) and risks suffering from scope creep and overly complex explanations. Key decisions needed:

- **Code detail level**: Should chapters include full implementations or high-level explanations?
- **Example strategy**: Copy-paste ready? Or illustrative snippets?
- **Implementation depth**: Teach algorithms or system integration?
- **Progressive complexity**: How much can each chapter assume?

Key constraints:
- Target audience: beginners with 0–3 years robotics experience
- Book chapters are 800–1200 words (10–12 minutes read)—not enough for deep dives
- Some domains (RL, LLMs) are complex; oversimplification risks undermining credibility
- Code examples must actually run (no pseudo-code that doesn't execute)

## Decision

**Chosen Approach: Progressive Implementation Depth (Conceptual → Practical → Integrated)**

### Strategy by Chapter:

**Chapter 1 (ROS 2): Full Working Examples**
- Provide complete, runnable Python publisher node (copy-paste ready)
- Include full URDF file (simple humanoid structure)
- Include launch file template
- All examples must run without modification
- Assumption: Reader has Python knowledge, can install ROS 2

**Chapter 2 (Digital Twin): Full Working Examples**
- Provide SDF file (Gazebo version of URDF)
- Provide launch file for Gazebo + RViz
- Provide Unity scene configuration (if applicable)
- All examples execute step-by-step as written

**Chapter 3 (Isaac): High-Level Conceptual, No Implementation Code**
- Explain USD format conceptually (no URDF → USD conversion code)
- Explain domain randomization strategy (no training code)
- Explain VSLAM/stereo depth/detection conceptually (no algorithm math)
- Explain Nav2 concepts (global + local planning, no planner code)
- Explain RL basics (reward, policy, training loop—no training loop code)
- Rationale: These topics are too complex for 800 words; focus on "why" and "what," not "how"

**Chapter 4 (VLA): Pseudocode + System Integration**
- Show ROS 2 action pseudocode (how to call nav2/manipulation actions)
- Show LLM task decomposition output as JSON (not actual LLM code)
- Show vision inference structure (not actual model training)
- Rationale: Actual LLM training/fine-tuning is out of scope; focus on how pieces integrate

### Code Example Philosophy:
- **Chapters 1–2**: "Fully functional and testable"
- **Chapters 3–4**: "Illustrative and conceptual; shows system structure, not implementation"
- **No pseudo-code in Ch1–2**: All code must execute
- **Pseudo-code OK in Ch3–4**: Explains workflow without implementation overhead
- **Documentation over implementation**: Code serves to illustrate architecture, not teach implementation

## Consequences

### Positive:
- **Beginner-friendly**: Early chapters are immediately practical; later chapters don't intimidate with advanced algorithms
- **Sustainable scope**: Avoids rabbit holes of deep algorithm explanations
- **Clear learning path**: Readers see systems working (Ch1–2) before learning theory (Ch3–4)
- **Focus on integration**: Readers understand how components work together, not internal mechanics
- **Testable outcomes**: Readers can verify learning by running code in Ch1–2

### Negative:
- **Advanced readers may feel bored**: Chapters 3–4 skip algorithm details (mitigated by "Further Reading" appendix)
- **Maintenance burden**: Chapters 1–2 code examples must be updated if dependencies change (mitigated by Docker Appendix)
- **Trust loss if code breaks**: If provided examples don't run, credibility suffers
- **Scope drift risk**: Temptation to add more code examples to feel "complete"

### Risks:
- **Version obsolescence**: Python/ROS 2/Gazebo updates break examples (mitigated by pinning to ROS 2 Humble LTS)
- **Dependencies missing**: Reader doesn't have Gazebo/Isaac installed; examples don't run (mitigated by comprehensive Appendix A)

## Alternatives Considered

### Alternative 1: No Code Examples (Pure Conceptual)
- **Approach**: All chapters are narrative + diagrams, no working code
- **Pros**: Minimal maintenance, no dependency issues, focuses on understanding
- **Cons**: Readers can't validate learning; feels incomplete; robotics needs hands-on elements
- **Rejected because**: Code is critical for beginner engagement; without examples, learning is passive

### Alternative 2: Full Implementation Code for All Chapters
- **Approach**: Chapters 1–4 all include complete, runnable implementations
- **Pros**: Hands-on throughout; readers see full system
- **Cons**: 800-word chapters can't fit RL training code or LLM fine-tuning; risks overwhelming beginners; RL/LLM training not in scope
- **Rejected because**: Scope explosion; later chapters become 3000+ words; detracts from beginner focus

### Alternative 3: Code in Separate Repository (Decoupled from Book)
- **Approach**: Book chapters reference external code repo; readers clone and run separately
- **Pros**: Code stays synchronized; easier to maintain and test; separates concerns
- **Cons**: Adds friction (must clone repo, find right directory); breaks narrative flow; readers may miss examples
- **Rejected because**: We want examples integrated into chapters for maximum engagement; external repo still exists (Appendix B reference)

### Alternative 4: Interactive Code Editor (Embedded in Docusaurus)
- **Approach**: Use something like StackBlitz or Monaco editor; readers run code in-browser
- **Pros**: No setup needed; immediate feedback; engagement
- **Cons**: Complex setup; limited to languages that run in-browser (not ROS 2); significant infrastructure
- **Rejected because**: ROS 2 and Gazebo can't run in-browser; adds complexity; beyond scope

## Implementation Plan

1. **Validate Chapter 1 examples**:
   - Test publisher node in ROS 2 Humble environment
   - Verify URDF loads in RViz without errors
   - Verify launch file executes without missing dependencies
   - Document exact commands readers should run

2. **Validate Chapter 2 examples**:
   - Test SDF file in Gazebo (launch, physics, sensors)
   - Test launch file for Gazebo + RViz combo
   - Verify Unity visualization receives and displays data

3. **Review Chapter 3 pseudocode**:
   - Verify pseudocode is clear and instructive
   - Ensure no actual code is provided (avoid maintenance burden)
   - Verify conceptual explanations don't oversimplify

4. **Review Chapter 4 examples**:
   - Verify ROS 2 action pseudocode is idiomatically correct
   - Verify JSON task decomposition example is realistic
   - Verify vision inference structure shows actual data types

5. **Create Docker image** (Appendix A support):
   - Dockerfile with ROS 2 Humble, Gazebo, Python 3.9+
   - Pre-installed dependencies (rclpy, Gazebo plugins, visualization tools)
   - Readers can `docker run` and have working environment

## Validation Criteria

- ✅ Chapter 1–2 code examples run without errors when following provided instructions
- ✅ Chapter 3–4 pseudocode is clear and doesn't require code knowledge to understand
- ✅ All examples follow best practices (error handling, comments, variable naming)
- ✅ Chapter 1–2 examples include expected output (for verification)
- ✅ Docker image builds and includes all prerequisites
- ✅ Zero "code will be added later" placeholders in final chapters

## References

- **Plan**: `specs/001-ros2-chapter/plan.md` (PART 2: Detailed Chapter Structure, PART 6: Architectural Decisions – 6.1)
- **Constitution**: `.specify/memory/constitution.md` (Principle: "All code samples must be correct and tested")
- **Module 1 Spec**: `specs/001-ros2-chapter/spec.md` (FR-003: "provide a complete, working Python publisher node")
- **Module 2 Spec**: `specs/002-digital-twin/spec.md` (FR-004: "provide example SDF or URDF file...")
