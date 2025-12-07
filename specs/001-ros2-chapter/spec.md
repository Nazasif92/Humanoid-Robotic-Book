# Feature Specification: ROS 2 — Robotic Nervous System

**Feature Branch**: `001-ros2-chapter`
**Created**: 2025-12-05
**Status**: Draft
**Input**: Module 1: ROS 2 — Robotic Nervous System (800-1200 words, beginner-focused)

## Objectives

Readers will understand ROS 2 as the communication and coordination backbone of humanoid robots. After reading this chapter, they can:
- Explain what ROS 2 is and why humanoid robots need it
- Understand nodes, topics, services, and actions as fundamental communication patterns
- Create a basic ROS 2 package and write a simple publisher/subscriber node
- Understand URDF as a way to describe humanoid robot structure
- Integrate URDF with ROS 2 tooling (visualization, simulation)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Understand ROS 2 Concepts (Priority: P1)

**Who**: Beginner learning humanoid robotics for the first time
**What**: Understand core ROS 2 concepts and why they matter for humanoid robots
**Why this priority**: Foundation for all other ROS 2 work; all other stories depend on this conceptual understanding

**Independent Test**: Reader can explain in their own words what a ROS 2 node is, what topics are, and how publisher/subscriber communication works. This can be tested by asking conceptual questions or having them write a 1-2 sentence explanation.

**Acceptance Scenarios**:

1. **Given** a reader has no prior ROS 2 experience, **When** they read the chapter introduction and architecture section, **Then** they understand what ROS 2 is and why humanoid robots use it (e.g., "ROS 2 is a framework that lets different robot parts communicate with each other")
2. **Given** a reader, **When** they see diagrams and explanations of nodes, topics, and services, **Then** they can distinguish between these concepts and explain their purpose (e.g., "A topic is one-way communication; a service is two-way request/response")
3. **Given** a reader, **When** they complete the chapter, **Then** they understand Quality of Service (QoS) concepts at a basic level without needing to configure it manually

---

### User Story 2 - Create a First ROS 2 Package (Priority: P1)

**Who**: Beginner ready to write their first code
**What**: Create a minimal ROS 2 package and write a simple publisher node that sends data
**Why this priority**: Practical foundation; readers need working code before advancing to complex topics

**Independent Test**: Reader successfully runs a Python script that publishes a message to a ROS 2 topic. This can be tested by: (1) providing setup instructions, (2) having them run the code, (3) asking them to modify the message to verify understanding.

**Acceptance Scenarios**:

1. **Given** a reader, **When** they follow the step-by-step package creation instructions, **Then** they can create a new ROS 2 package without errors (including `package.xml` and directory structure)
2. **Given** a template publisher node in rclpy, **When** they understand the code with inline comments explaining each part, **Then** they can modify the message or publication rate without breaking the node
3. **Given** a running publisher node, **When** they run the provided ROS 2 CLI command, **Then** they can see messages being published (e.g., `ros2 topic echo /topic_name`)

---

### User Story 3 - Understand URDF for Humanoid Robots (Priority: P2)

**Who**: Beginner wanting to model a humanoid robot structure
**What**: Learn URDF syntax for describing a simple humanoid structure (links, joints, sensors)
**Why this priority**: Essential for visualization and simulation; builds on core concepts but not needed for basic pub/sub

**Independent Test**: Reader can read a simple URDF file and identify which parts are links, which are joints, and what a sensor definition looks like. This can be tested by providing an annotated URDF and asking them to explain what each section does.

**Acceptance Scenarios**:

1. **Given** a well-commented URDF example for a simple humanoid arm, **When** they read the XML structure, **Then** they understand what `<link>`, `<joint>`, and `<inertial>` elements represent (without needing to configure inertia values)
2. **Given** a URDF file with sensor definitions, **When** they see a camera sensor in the head, **Then** they understand how sensors are attached to links (conceptually, not implementation details)
3. **Given** the chapter instructions, **When** they visualize a simple URDF using `rviz2`, **Then** they see the robot model rendered and can understand the relationship between URDF XML and the visual representation

---

### User Story 4 - Integrate URDF with ROS 2 Tooling (Priority: P2)

**Who**: Beginner ready to visualize and use URDF in the ROS 2 ecosystem
**What**: Load a URDF file in ROS 2, visualize it, and understand the integration workflow
**Why this priority**: Completes the bridge between static description (URDF) and live system (ROS 2); enables simulation and visualization

**Independent Test**: Reader successfully launches RViz2, loads a URDF file, and sees the robot model displayed. This can be tested by providing launch file templates and step-by-step instructions.

**Acceptance Scenarios**:

1. **Given** a launch file template that loads a URDF, **When** they run `ros2 launch`, **Then** RViz2 opens and displays the robot model correctly
2. **Given** a running URDF in RViz2, **When** they understand the relationship between `/robot_description` topic and the visualization, **Then** they grasp how ROS 2 communicates static data (like URDF) via topics
3. **Given** the completed chapter, **When** they review the workflow diagram (source → URDF → ROS 2 parameter → RViz2), **Then** they understand how all pieces connect

---

### Edge Cases

- What happens when a reader has ROS 1 experience? (Note: ROS 2 is significantly different; chapter should highlight key differences briefly)
- How does the chapter handle readers who don't have a Linux/ROS 2 environment set up? (Recommend Docker or WSL; provide brief setup instructions in a sidebar, not main chapter)
- What if a reader's URDF doesn't load in RViz2? (Provide minimal troubleshooting: common XML syntax errors, missing dependencies)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Chapter MUST explain ROS 2 architecture using clear, beginner-friendly language (e.g., "messaging system for robot parts")
- **FR-002**: Chapter MUST include diagrams illustrating nodes, topics, services, and actions (ASCII art or SVG acceptable; must be clear)
- **FR-003**: Chapter MUST provide a complete, working Python publisher node using rclpy that readers can copy-paste and run
- **FR-004**: Chapter MUST explain how to create a ROS 2 package with `ros2 pkg create` command and provide example output
- **FR-005**: Chapter MUST include a fully commented URDF example for a simple humanoid robot (e.g., torso + 2 arms + 1 head)
- **FR-006**: Chapter MUST explain URDF elements (links, joints, inertia, collision, visual) without requiring readers to calculate inertia values
- **FR-007**: Chapter MUST provide a launch file example that loads URDF and starts RViz2
- **FR-008**: Chapter MUST explain Quality of Service (QoS) concepts at a high level (reliability, durability) without requiring readers to configure custom QoS policies
- **FR-009**: Chapter MUST verify all code examples run without errors using Python 3.9+ and ROS 2 Humble (or later)
- **FR-010**: Chapter MUST cite official ROS 2 documentation for all major concepts (links to docs.ros.org)

### Key Entities *(data-driven aspects)*

- **ROS 2 Node**: A computational unit that runs as a separate process; communicates via topics/services/actions
- **Topic**: A named bus for pub/sub communication; one-way data stream
- **Service**: Request/response communication pattern; synchronous two-way interaction
- **Action**: Long-running tasks with feedback; used for goals like motion control
- **URDF (Unified Robot Description Format)**: XML file describing robot structure (links, joints, sensors, visual/collision properties)
- **Link**: A rigid body in a robot (e.g., arm segment, torso); has mass, shape, inertia
- **Joint**: Connection between two links; defines relative motion (revolute, prismatic, fixed)
- **Launch File**: ROS 2 configuration file (Python or XML) that starts multiple nodes, loads parameters, and configures the system

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Reader understands ROS 2 concepts (nodes, topics, services, actions) without implementation experience; can explain in 1-2 sentences per concept
- **SC-002**: Reader successfully creates a ROS 2 package and runs a provided publisher node without errors (verified by running the code in their environment)
- **SC-003**: Reader can read and understand a URDF file; can identify links, joints, and sensors in an annotated example
- **SC-004**: Reader can visualize a URDF in RViz2 using provided launch file and understand the integration workflow
- **SC-005**: All code examples (Python nodes, URDF files, launch files) are correct, tested, and runnable in ROS 2 Humble or later on Linux
- **SC-006**: Chapter adheres to book constitution: accurate technical content, beginner-friendly language, consistent formatting, clear examples
- **SC-007**: Chapter length: 800–1200 words (approximately 8–12 minutes reading time)
- **SC-008**: All diagrams are clear and correctly illustrate the concepts (nodes communicating via topics, URDF structure, ROS 2 ecosystem integration)
- **SC-009**: Zero technical errors; all ROS 2 commands and API calls are current (no deprecated methods)
- **SC-010**: References to external resources (docs.ros.org, GitHub, packages) are active and up-to-date

## Assumptions

- Readers have basic Python knowledge (functions, variables, imports) but no prior ROS 2 experience
- Readers have access to a Linux environment (bare metal, VM, Docker, or WSL2) with ROS 2 installed
- Setup instructions will be brief; a separate appendix or prerequisite chapter covers full environment setup
- Diagrams will be created as ASCII art or simple SVG; no need for animations or interactive visualizations
- ROS 2 version assumed: Humble (LTS, recommended for learning)

## Out of Scope

- Deep dive into C++ ROS 2 programming (Python only for this chapter)
- Advanced motion control algorithms or kinematics
- Custom QoS policy configuration (only high-level concepts)
- Performance tuning or optimization techniques
- ROS 1 to ROS 2 migration details (only a brief note on key differences)
- Advanced URDF features (plugin descriptions, gazebo tags, complex sensors)
- Real-time constraints or deterministic behavior
