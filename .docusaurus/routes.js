import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '567'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'f65'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '1a8'),
            routes: [
              {
                path: '/docs/appendices/code-examples',
                component: ComponentCreator('/docs/appendices/code-examples', 'fd7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/appendices/environment-setup',
                component: ComponentCreator('/docs/appendices/environment-setup', 'bf0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/appendices/further-reading',
                component: ComponentCreator('/docs/appendices/further-reading', 'c5d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/appendices/glossary',
                component: ComponentCreator('/docs/appendices/glossary', 'aed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapters/digital-twin-simulation',
                component: ComponentCreator('/docs/chapters/digital-twin-simulation', '9f0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapters/isaac-perception',
                component: ComponentCreator('/docs/chapters/isaac-perception', '1f2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapters/ros2-nervous-system',
                component: ComponentCreator('/docs/chapters/ros2-nervous-system', '6ab'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapters/vla-humanoid',
                component: ComponentCreator('/docs/chapters/vla-humanoid', '10b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/skill/intro',
                component: ComponentCreator('/docs/skill/intro', 'b20'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/skill/lesson-1/',
                component: ComponentCreator('/docs/skill/lesson-1/', '495'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/skill/lesson-2/',
                component: ComponentCreator('/docs/skill/lesson-2/', '6f9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/skill/lesson-3/',
                component: ComponentCreator('/docs/skill/lesson-3/', '9d9'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
