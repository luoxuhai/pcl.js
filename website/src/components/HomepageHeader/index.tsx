import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { translate } from '@docusaurus/Translate';

import styles from './index.module.css';

function HomepageHeader() {
    const { siteConfig } = useDocusaurusContext();
    return (
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className={clsx('container', styles.container)}>
          <h1 className="hero__title">{siteConfig.title}</h1>
          <p className="hero__subtitle">
            {translate({
              message: 'home.tagline',
            })}
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/tutorials/intro"
            >
              {translate({
                message: 'home.getting-started',
              })}
            </Link>
          </div>
        </div>
      </header>
    );
}

export default HomepageHeader;
