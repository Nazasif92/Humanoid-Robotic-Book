import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/humanoid-robotic-book/__docusaurus/debug',
    component: ComponentCreator('/humanoid-robotic-book/__docusaurus/debug', '862'),
    exact: true
  },
  {
    path: '/humanoid-robotic-book/__docusaurus/debug/config',
    component: ComponentCreator('/humanoid-robotic-book/__docusaurus/debug/config', '5db'),
    exact: true
  },
  {
    path: '/humanoid-robotic-book/__docusaurus/debug/content',
    component: ComponentCreator('/humanoid-robotic-book/__docusaurus/debug/content', '7fd'),
    exact: true
  },
  {
    path: '/humanoid-robotic-book/__docusaurus/debug/globalData',
    component: ComponentCreator('/humanoid-robotic-book/__docusaurus/debug/globalData', '13f'),
    exact: true
  },
  {
    path: '/humanoid-robotic-book/__docusaurus/debug/metadata',
    component: ComponentCreator('/humanoid-robotic-book/__docusaurus/debug/metadata', '571'),
    exact: true
  },
  {
    path: '/humanoid-robotic-book/__docusaurus/debug/registry',
    component: ComponentCreator('/humanoid-robotic-book/__docusaurus/debug/registry', '78f'),
    exact: true
  },
  {
    path: '/humanoid-robotic-book/__docusaurus/debug/routes',
    component: ComponentCreator('/humanoid-robotic-book/__docusaurus/debug/routes', 'bd8'),
    exact: true
  },
  {
    path: '/humanoid-robotic-book/docs',
    component: ComponentCreator('/humanoid-robotic-book/docs', '151'),
    routes: [
      {
        path: '/humanoid-robotic-book/docs',
        component: ComponentCreator('/humanoid-robotic-book/docs', 'c3f'),
        routes: [
          {
            path: '/humanoid-robotic-book/docs',
            component: ComponentCreator('/humanoid-robotic-book/docs', 'd71'),
            routes: [
              {
                path: '/humanoid-robotic-book/docs/appendices/code-examples',
                component: ComponentCreator('/humanoid-robotic-book/docs/appendices/code-examples', '2f0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid-robotic-book/docs/appendices/environment-setup',
                component: ComponentCreator('/humanoid-robotic-book/docs/appendices/environment-setup', 'ea9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid-robotic-book/docs/appendices/further-reading',
                component: ComponentCreator('/humanoid-robotic-book/docs/appendices/further-reading', '2ff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid-robotic-book/docs/appendices/glossary',
                component: ComponentCreator('/humanoid-robotic-book/docs/appendices/glossary', '71d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid-robotic-book/docs/chapters/digital-twin-simulation',
                component: ComponentCreator('/humanoid-robotic-book/docs/chapters/digital-twin-simulation', 'ff8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid-robotic-book/docs/chapters/isaac-perception',
                component: ComponentCreator('/humanoid-robotic-book/docs/chapters/isaac-perception', '1d2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid-robotic-book/docs/chapters/ros2-nervous-system',
                component: ComponentCreator('/humanoid-robotic-book/docs/chapters/ros2-nervous-system', '732'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid-robotic-book/docs/chapters/vla-humanoid',
                component: ComponentCreator('/humanoid-robotic-book/docs/chapters/vla-humanoid', '0b6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid-robotic-book/docs/intro',
                component: ComponentCreator('/humanoid-robotic-book/docs/intro', '036'),
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
    path: '/humanoid-robotic-book/',
    component: ComponentCreator('/humanoid-robotic-book/', '9a7'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
