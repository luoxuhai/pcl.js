import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Link from '@docusaurus/Link';
import { translate } from '@docusaurus/Translate';
import clsx from 'clsx';

import styles from './index.module.scss';

const exampleList = [
  {
    title: translate({
      message: 'examples.statistical-outlier-removal',
    }),
    cover: require('@site/static/img/examples/StatisticalOutlierRemoval.gif')
      .default,
    code_url: 'https://kl2zjs.csb.app',
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
          return (
            <Link
              className={styles.cell}
              href={example.code_url}
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
