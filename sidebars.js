// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro', // Introduction page
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 — Robotic Nervous System',
      items: [
        'chapters/ros2-nervous-system',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo + Unity — Digital Twin',
      items: [
        'chapters/digital-twin-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac — AI Brain & Perception',
      items: [
        'chapters/isaac-perception',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA — Vision, Language, Action',
      items: [
        'chapters/vla-humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Skill',
      items: [
        'skill/intro',
        'skill/lesson-1/index',
        'skill/lesson-2/index',
        'skill/lesson-3/index',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/environment-setup',
        'appendices/code-examples',
        'appendices/glossary',
        'appendices/further-reading',
      ],
    },
  ],
};

module.exports = sidebars;