import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  roboticsSidebar: [
    'index',
    {
      type: 'category',
      label: 'Chapters',
      items: [
        'Chapters/chapter-01-state-representations/index',
        'Chapters/chapter-02-modelling/index',
        'Chapters/chapter-03-control/index',
        'Chapters/chapter-04-state-estimation/index',
        'Chapters/chapter-05-navigation/index',
        'Chapters/chapter-06-kinematics/index',
        'Chapters/chapter-07-dynamics/index',
        'Chapters/chapter-08-perception-and-learning/index',
        'Chapters/chapter-09-rl/index',
        'Chapters/chapter-10-hri/index',
      ],
    },
    'glossary',
  ],
};

export default sidebars;
