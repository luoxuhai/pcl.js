import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';
import { translateMsg } from '@site/src/utils/common';

type FeatureItem = {
  title: string;
  cover: string;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: translateMsg('home.feature.title.typescript'),
    cover:
      'https://ionicframework.com/_next/image?url=https%3A%2F%2Fimages.prismic.io%2Fionicframeworkcom%2F79d0bc82-4833-4efc-9902-91ba30d3e767_framework-homepage-top-0.png&w=96&q=75',
    description: <>{translateMsg('home.feature.description.typescript')}</>,
  },
  {
    title: translateMsg('home.feature.title.support'),
    cover:
      'https://ionicframework.com/_next/image?url=https%3A%2F%2Fimages.prismic.io%2Fionicframeworkcom%2Fb77c801b-a779-4cf6-9851-a92faf0c977d_framework-homepage-top-1.png&w=96&q=75',
    description: <>{translateMsg('home.feature.description.support')}</>,
  },
  {
    title: translateMsg('home.feature.title.api'),
    cover:
      'https://ionicframework.com/_next/image?url=https%3A%2F%2Fimages.prismic.io%2Fionicframeworkcom%2Ff42e2b98-8ed8-45d2-b790-d0dd2dd49ec8_framework-homepage-top-2.png&w=96&q=75',
    description: <>{translateMsg('home.feature.description.api')}</>,
  },
];

const examples = [
  {
    title: translateMsg('home.example.preprocess'),
    cover: require('@site/static/img/examples/StatisticalOutlierRemoval.jpg').default,
    url: 'https://kl2zjs.csb.app/',
  },
  {
    title: translateMsg('home.example.register'),
    cover: require('@site/static/img/examples/IterativeClosestPoint.jpg').default,
    url: 'https://1t72c1.csb.app/',
  },
  {
    title: translateMsg('home.example.segmentation'),
    cover: require('@site/static/img/examples/MinCutSegmentation.jpg').default,
    url: 'https://o4y07f.csb.app/',
  },
  {
    title: translateMsg('home.example.keypoints'),
    cover: require('@site/static/img/examples/ISSKeypoint3D.jpg').default,
    url: 'https://3l6tfj.csb.app/',
  },
];

function Feature({ title, cover, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <img className={styles.featureSvg} src={cover} />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p className={styles.description}>{description}</p>
      </div>
    </div>
  );
}

function Example({ title, cover, url }) {
  return (
    <div className={clsx('col col--3', styles.example)}>
      <Link className="text--center" to={url}>
        <img className={clsx('card shadow--sx', styles.exampleImg)} src={cover} />
      </Link>
      <div className="text--center padding-horiz--md">
        <h3
          style={{
            marginTop: 20,
          }}
        >
          {title}
        </h3>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>

        <div
          className="row"
          style={{
            marginTop: 120,
          }}
        >
          {examples.map((props, idx) => (
            <Example key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
