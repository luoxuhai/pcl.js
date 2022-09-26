import React, { useCallback, useEffect, useRef } from 'react';
import clsx from 'clsx';
import {
  WebGLRenderer,
  Scene,
  PerspectiveCamera,
  PointsMaterial,
  Points,
  BufferGeometry,
} from 'three';
import { PCDLoader } from 'three/examples/jsm/loaders/PCDLoader';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { translate } from '@docusaurus/Translate';

import styles from './index.module.css';

const h = 500;
let timer: any;

function BackgroundCanvas() {
  const renderer = useRef<WebGLRenderer>(null);
  const scene = useRef<Scene>(null);
  const camera = useRef<PerspectiveCamera>(null);
  const cloudWolf = useRef<Points<BufferGeometry, PointsMaterial>>(null);
  const cloudHorse = useRef<Points<BufferGeometry, PointsMaterial>>(null);

  useEffect(() => {
    const width = window.innerWidth;
    renderer.current = new WebGLRenderer({
      powerPreference: 'high-performance',
      canvas: document.getElementById('bg-canvas'),
      antialias: true,
    });
    renderer.current.setPixelRatio(window.devicePixelRatio);

    camera.current = new PerspectiveCamera(45, width / h, 0.01, 10000);
    scene.current = new Scene();
    onResize();
    addPCDToScene();
    renderer.current.setAnimationLoop(render);

    function render() {
      const time = Date.now() * 0.001;
      if (cloudWolf.current) {
        cloudWolf.current.rotation.y = Math.sin(time / 4);
        cloudWolf.current.rotation.z = Math.sin(time / 2);
      }
      if (cloudHorse.current) {
        cloudHorse.current.rotation.z = -Math.sin(time / 4);
      }

      renderer.current.render(scene.current, camera.current);
    }

    return () => {
      renderer.current.dispose();
      clearInterval(timer);
    };
  }, []);

  useEffect(() => {
    window.addEventListener('resize', onResize);

    return () => {
      window.removeEventListener('resize', onResize);
    };
  });

  const addPCDToScene = useCallback(async () => {
    const _cloudWolf = (await new PCDLoader().loadAsync(
      'https://3l6tfj.csb.app/ism_test_wolf.pcd',
    )) as Points<BufferGeometry, PointsMaterial>;
    const _cloudHorse = (await new PCDLoader().loadAsync(
      'https://3l6tfj.csb.app/ism_train_horse.pcd',
    )) as Points<BufferGeometry, PointsMaterial>;

    cloudWolf.current = _cloudWolf;
    cloudHorse.current = _cloudHorse;

    [_cloudWolf, _cloudHorse].forEach((cloud) => {
      cloud.geometry.center();
      cloud.material.sizeAttenuation = false;
      cloud.material.size = 1.25;
      scene.current.add(cloud);
      cloud.rotation.x = -Math.PI / 2;
      cloud.rotation.z = Math.PI / 4;
    });

    _cloudWolf.position.x = 150;
    _cloudHorse.position.x = -250;
    _cloudHorse.position.z = -200;
    camera.current.position.z = 250;
    timer = setInterval(() => {
      [_cloudWolf, _cloudHorse].forEach((cloud) => {
        cloud.material.color.setHex(Math.random() * 0xffffff);
      });
    }, 1000);
  }, []);

  const onResize = useCallback(() => {
    camera.current.aspect = window.innerWidth / h;
    camera.current.updateProjectionMatrix();
    renderer.current.setSize(window.innerWidth, h);
  }, []);

  return <canvas id="bg-canvas" className={styles.canvas} />;
}

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header
      className={clsx('hero hero--primary', styles.heroBanner)}
      style={{
        height: h,
      }}
    >
      <div className={clsx('container', styles.container)}>
        <h1 className={clsx('hero__title', styles.title)}>
          {siteConfig.title}
        </h1>
        <p className={clsx('hero__subtitle', styles.desc)}>
          {translate({
            message: 'home.tagline',
          })}
        </p>
        <div className={styles.buttons}>
          <Link
            className={clsx('button button--secondary button--lg', styles.button)}
            to="/docs/tutorials/intro"
          >
            {translate({
              message: 'home.getting-started',
            })}
          </Link>
        </div>
      </div>
      <BackgroundCanvas />
    </header>
  );
}

export default HomepageHeader;
