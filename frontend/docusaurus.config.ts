import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI Textbook',
  tagline: 'Humanoid Robotics with ROS2, Gazebo, NVIDIA Isaac & VLA',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // GitHub Pages deployment
  url: 'https://EnggQasim.github.io',
  baseUrl: '/physical-ai-robotics-textbook/',
  organizationName: 'EnggQasim',
  projectName: 'physical-ai-robotics-textbook',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
    },
  },

  themes: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        language: ['en'],
        highlightSearchTermsOnTargetPage: true,
        docsRouteBasePath: '/docs',
        indexBlog: false,
      },
    ],
  ],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/EnggQasim/physical-ai-robotics-textbook/tree/main/frontend/',
        },
        blog: false, // Disable blog for textbook
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/social-card.jpg',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Textbook',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Chapters',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/EnggQasim/physical-ai-robotics-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Chapters',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'ROS2 Fundamentals',
              to: '/docs/module-1-ros2',
            },
            {
              label: 'Robot Simulation',
              to: '/docs/module-2-simulation',
            },
          ],
        },
        {
          title: 'More Chapters',
          items: [
            {
              label: 'NVIDIA Isaac',
              to: '/docs/module-3-nvidia-isaac',
            },
            {
              label: 'Vision-Language-Action',
              to: '/docs/module-4-vla',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/EnggQasim/physical-ai-robotics-textbook',
            },
            {
              label: 'Panaversity',
              href: 'https://panaversity.org',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built for Panaversity Hackathon.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'cpp', 'yaml', 'bash', 'json'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
