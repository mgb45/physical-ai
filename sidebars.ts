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
      label: 'Course Content',
      items: [
        'Weeks/week-01-state-representations/index',
        'Weeks/week-02-modelling/index',
        'Weeks/week-03-control/index',
        'Weeks/week-04-state-estimation/index',
        'Weeks/week-05-navigation/index',
        'Weeks/week-06-kinematics/index',
        'Weeks/week-07-dynamics/index',
        'Weeks/week-08-perception-and-learning/index',
        'Weeks/week-09-rl/index',
        'Weeks/week-10-hri/index',
      ],
    },
    'glossary',
  ],
};

export default sidebars;
