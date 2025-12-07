import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Module 1: ROS 2 — Robotic Nervous System',
    description: (
      <>
        Learn ROS 2 fundamentals: nodes, topics, services, URDF, and integration tooling.
        Understand how ROS 2 serves as the communication backbone for humanoid robots.
      </>
    ),
    to: '/docs/chapters/ros2-nervous-system'
  },
  {
    title: 'Module 2: Digital Twin Simulation',
    description: (
      <>
        Explore Gazebo and Unity integration for creating digital twins of humanoid robots.
        Learn to simulate robot behavior in virtual environments.
      </>
    ),
    to: '/docs/chapters/digital-twin-simulation'
  },
  {
    title: 'Module 3: AI Perception & Control',
    description: (
      <>
        Master NVIDIA Isaac for AI perception and planning in humanoid robotics.
        Implement computer vision and sensor processing algorithms.
      </>
    ),
    to: '/docs/chapters/isaac-perception'
  },
  {
    title: 'Module 4: Vision-Language-Action',
    description: (
      <>
        Develop multimodal autonomous systems using Vision-Language-Action paradigms.
        Create robots that can see, understand, and act in complex environments.
      </>
    ),
    to: '/docs/chapters/vla-humanoid'
  },
  {
    title: 'Beginner Friendly',
    description: (
      <>
        Designed for learners with step-by-step tutorials, code examples,
        and clear explanations of complex robotics concepts.
      </>
    ),
    to: '/docs/intro'
  },
  {
    title: 'Hands-On Approach',
    description: (
      <>
        Practical examples and exercises that help you build real robotics applications
        with ROS 2, simulation tools, and AI frameworks.
      </>
    ),
    to: '/docs/appendices/environment-setup'
  }
];

function Feature({title, description, to}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
        <Link className="button button--secondary button--sm" to={to}>
          Learn More
        </Link>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}