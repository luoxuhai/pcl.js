import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Link from '@docusaurus/Link';
import Translate, { translate } from '@docusaurus/Translate';
import clsx from 'clsx';

import styles from './index.module.scss';

const commonOptions =
  'file=page2.html&hideExplorer=1&hideNavigation=1&view=preview';

const exampleList = [
  {
    title: translate({
      message: 'examples.statistical-outlier-removal',
    }),
    cover: require('@site/static/img/filters_small.jpg').default,
    code_url: 'https://stackblitz.com/edit/web-platform-ugzuzp?embed=1',
  },
];

export default function Examples() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={siteConfig.title}
      description="Description will go into a meta tag in <head />"
    >
      <div className={styles.list}>
        {exampleList.map((example) => {
          const url = example.code_url
            ? encodeURIComponent(`${example.code_url}&${commonOptions}`)
            : '';

          return (
            <Link
              className={styles.cell}
              to={`examples/detail?code_url=${url}`}
              key={example.title}
            >
              <div className={clsx('card shadow--sx', styles.content)}>
                <img src={example.cover} title={example.title} />
                <p>{example.title}</p>
              </div>
            </Link>
          );
        })}
      </div>
    </Layout>
  );
}
