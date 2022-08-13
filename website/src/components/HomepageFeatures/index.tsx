import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  cover: string;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Filters',
    cover: require('@site/static/img/filters_small.jpg').default,
    description: (
      <>
        Some of these outliers can be filtered by performing a statistical
        analysis on each point’s neighborhood.
      </>
    ),
  },
  {
    title: 'Keypoints',
    cover: require('@site/static/img/keypoints_small.jpg').default,
    description: (
      <>
        The keypoints library contains implementations of two point cloud
        keypoint detection algorithms.
      </>
    ),
  },
  {
    title: 'Registration',
    cover: require('@site/static/img/registration_small.jpg').default,
    description: (
      <>
        Combining several datasets into a global consistent model is usually
        performed using a technique called registration.
      </>
    ),
  },
  {
    title: 'Kdtree',
    cover: require('@site/static/img/kdtree_mug.jpg').default,
    description: (
      <>
        A Kd-tree (k-dimensional tree) is a space-partitioning data structure that
        enables efficient range searches and nearest neighbor searches.
      </>
    ),
  },
  {
    title: 'Octree',
    cover: require('@site/static/img/octree_bunny.jpg').default,
    description: (
      <>
        The octree library provides efficient methods for creating a
        hierarchical tree data structure from point cloud data.
      </>
    ),
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

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
