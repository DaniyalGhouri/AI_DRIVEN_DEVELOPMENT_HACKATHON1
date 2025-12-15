import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A Comprehensive Academic Textbook',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },


  // Set the production url of your site here
  url: 'https://user.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/physical-ai-humanoid-robotics-textbook/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'user', // Usually your GitHub org/user name.
  projectName: 'physical-ai-humanoid-robotics-textbook', // Usually your repo name.

  onBrokenLinks: 'warn', // Return to strict checking now that links are fixed

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: '/', // Keep docs at root to match existing links
          // Remove the "edit this page" links
          // editUrl: 'https://github.com/user/physical-ai-humanoid-robotics-textbook/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  // Add client modules to inject global components
  clientModules: [require.resolve('./src/theme/Root.js')],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'My Site Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'doc',
          docId: 'modules/module-1-ros2/introduction-to-ros2',
          position: 'left',
          label: 'Module 1: ROS 2',
        },
        {
          type: 'dropdown',
          label: 'Modules',
          position: 'left',
          items: [
            {
              type: 'doc',
              docId: 'modules/module-1-ros2/introduction-to-ros2',
              label: 'Module 1: ROS 2',
            },
            {
              type: 'doc',
              docId: 'modules/module-2-digital-twin/introduction-to-digital-twins',
              label: 'Module 2: Digital Twin',
            },
            {
              type: 'doc',
              docId: 'modules/module-3-isaac/introduction-to-nvidia-isaac-ecosystem',
              label: 'Module 3: AI-Robot Brain',
            },
            {
              type: 'doc',
              docId: 'modules/module-4-vla/understanding-vla-robotics',
              label: 'Module 4: VLA',
            },
          ],
        },
      ],
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
