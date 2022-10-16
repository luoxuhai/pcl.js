import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  cover: string;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'TypeScript 类型支持',
    cover:
      'https://ionicframework.com/_next/image?url=https%3A%2F%2Fimages.prismic.io%2Fionicframeworkcom%2F79d0bc82-4833-4efc-9902-91ba30d3e767_framework-homepage-top-0.png&w=96&q=75',
    description: <>使用 TypeScript 编写，具有可预测的静态类型</>,
  },
  {
    title: '支持所有现代浏览器',
    cover:
      'https://ionicframework.com/_next/image?url=https%3A%2F%2Fimages.prismic.io%2Fionicframeworkcom%2Fb77c801b-a779-4cf6-9851-a92faf0c977d_framework-homepage-top-1.png&w=96&q=75',
    description: <>支持所有现代浏览器，未来将提供对 Nodejs 的支持</>,
  },
  {
    title: '方便移植',
    cover:
      'https://ionicframework.com/_next/image?url=https%3A%2F%2Fimages.prismic.io%2Fionicframeworkcom%2Ff42e2b98-8ed8-45d2-b790-d0dd2dd49ec8_framework-homepage-top-2.png&w=96&q=75',
    description: <>提供与 PCL(C++) 相似的 API，简单易用</>,
  },
];

const examples = [
  {
    title: '点云预处理',
    cover: require('@site/static/img/examples/StatisticalOutlierRemoval.jpg').default,
    url: 'https://kl2zjs.csb.app/',
  },
  {
    title: '配准点云',
    cover: require('@site/static/img/examples/IterativeClosestPoint.jpg').default,
    url: 'https://1t72c1.csb.app/',
  },
  {
    title: '分割物体',
    cover: require('@site/static/img/examples/MinCutSegmentation.jpg').default,
    url: 'https://o4y07f.csb.app/',
  },
  {
    title: '提取关键点',
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
    <div className={clsx('col col--3', styles.example)} to={url}>
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

        <h3
          style={{
            textAlign: 'center',
            marginTop: 80,
            marginBottom: 80,
            fontSize: 38,
          }}
        >
          展示
        </h3>
        <div className="row">
          {examples.map((props, idx) => (
            <Example key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
