// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

const github = 'https://github.com/luoxuhai/pcl.js';
const url =
  process.env.NODE_ENV !== 'development'
    ? 'https://pcljs.org'
    : 'http://localhost:3000';
const title = 'PCL.js';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title,
  tagline: '☁️ Point Cloud Library (PCL) for browser, powered by WebAssembly.',
  url,
  baseUrl: '/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  organizationName: 'luoxuhai',
  projectName: 'pcl.js',
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'zh-cn'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      'zh-cn': {
        label: '简体中文',
        direction: 'ltr',
      },
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title,
        logo: {
          alt: 'My Site Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'doc',
            docId: 'intro',
            position: 'left',
            label: 'Documentation',
          },
          {
            label: 'API',
            position: 'left',
            to: '/api',
          },
          { label: 'Examples', position: 'left', to: '/examples' },
          {
            href: github,
            label: 'GitHub',
            position: 'right',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'light',
        links: [
          {
            title: 'About',
            items: [
              {
                label: 'Introduction',
                to: '/docs/introduction',
              },
              // {
              //   label: 'Powered by Vercel',
              // },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Issues',
                href: `${github}/issues`,
              },
              {
                label: 'Discussions',
                href: `${github}/discussions`,
              },
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/pcl.js',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/LuoXuhai',
              },
            ],
          },
          {
            title: 'Download',
            items: [
              {
                label: 'GitHub',
                href: github,
              },
              {
                label: 'NPM',
                href: 'https://www.npmjs.com/package/pcl.js',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Sponsor',
                href: github,
              },
            ],
          },
        ],
        copyright: `© ${new Date().getFullYear()} · Darkce · All rights reserved`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;
