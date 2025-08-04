// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'MIRMI - EnviroBotics',
  tagline: 'Summer School of Environmental Robotics',
  favicon: 'img/TUM_Logo.png',

  url: 'https://lennarttro.github.io',
  baseUrl: '/EnviroBotics-Docs/',

  organizationName: 'lennarttro',
  projectName: 'EnviroBotics-Docs',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: '/docs',
          sidebarPath: require.resolve('./sidebars.js'),
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/envirobotics-social-card.jpg', 
    navbar: {
      title: 'MIRMI - EnviroBotics',
      logo: {
        alt: 'EnviroBotics Logo',
        src: 'img/TUM_Logo.png', 
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Documentation',
        },
        {
          href: 'https://github.com/lennarttro/Summer-School-EnviroBotics',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'Overview',
              to: '/docs/overview and concept/overview',
            },
            {
              label: 'FAQ',
              to: '/docs/appendix/faq',
            },
          ],
        },
      ],
      copyright: `© ${new Date().getFullYear()} EnviroBotics – All rights reserved.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};

export default config;
