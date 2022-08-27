<p align="center">
  <h1 align="center">PCL.js</h1>
  <p align="center">用于浏览器的点云库 (<a href="https://github.com/PointCloudLibrary/pcl" target="_blank">PCL</a>)，由 WebAssembly 提供支持。</p>
</p>
<p align="center">
 <a href="https://github.com/FoalTS/foal/blob/master/LICENSE">
    <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT">
  </a>
 <a href="https://packagephobia.com/result?p=pcl.js">
    <img src="https://packagephobia.com/badge?p=pcl.js" alt="Bundle Size">
  </a>
    <a href="https://badge.fury.io/js/pcl.js">
    <img src="https://badge.fury.io/js/pcl.js.svg" alt="npm version">
  </a>
  <a href="https://www.npmtrends.com/pcl.js">
    <img src="https://img.shields.io/npm/dm/pcl.js" alt="Downloads" />
  </a>
  <a href="https://github.com/luoxuhai/pcl.js/stargazers">
    <img src="https://img.shields.io/github/stars/luoxuhai/pcl.js" alt="Github Stars" />
  </a>
  <a href="https://www.jsdelivr.com/package/npm/pcl.js">
    <img src="https://data.jsdelivr.com/v1/package/npm/pcl.js/badge?style=rounded" alt="jsDelivr" />
  </a>
  <a href="https://openbase.com/js/pcl.js?utm_source=embedded&amp;utm_medium=badge&amp;utm_campaign=rate-badge">
    <img src="https://badges.openbase.com/js/rating/pcl.js.svg?token=nF4Z9XUsUhOe5yeVDZTPwpdoKqqamFbVBoVA5zbU5iM=" alt="Rate this package" />
  </a>
  <a href="#badge">
    <img alt="semantic-release: angular" src="https://img.shields.io/badge/semantic--release-angular-e10079?logo=semantic-release">
  </a>
</p>

<p align="center">
  <a href="./README.md">English</a> | 简体中文
</p>

## 简介

***pcl.js*** 是在浏览器中运行的 [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl)，由 [Emscripten](https://emscripten.org/index.html) 和 [WebAssembly](https://webassembly.org/) 提供支持。[Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl) 是一个独立、大型、开源的 2D/3D 图像和点云处理库。

## 资源

- [文档](https://pcljs.org/docs)
- [API 参考](https://pcljs.org/api)
- [示例](https://pcljs.org/examples)

## 安装

### NPM

```bash
npm install pcl.js

or

yarn add pcl.js
```

### CDN

#### 开发环境

```html
<script src="https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.js"><script>
```

#### 生产环境

```html
<script src="https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.min.js"><script>
```

## 使用

### NPM

```typescript
import PCL from 'pcl.js';

async function main() {
  // 初始化
  const pcl = await PCL.init({
    // 推荐，可选配置，自定义 WebAssembly 文件链接
    url: 'https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm',
    // 也可以传人 WebAssembly 文件的 ArrayBuffer
    // arrayBuffer: ArrayBuffer
  });

  // ...
}

main();
```

### CDN

```html
<script>
async function main() {
  // 初始化，PCL 是全局对象
  const pcl = await PCL.init({
    // 推荐，可选配置，自定义 WebAssembly 文件链接
    url: 'https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm',
    // 也可以传人 WebAssembly 文件的 ArrayBuffer
    // arrayBuffer: ArrayBuffer
  });

  // ...
}

main();
</script>
```
### 简单示例
```typescript
import PCL from 'pcl.js';

async function main() {
  const pcl = await PCL.init({
    url: 'https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm',
  });

  // 获取 PCD 文件
  const pcd = await fetch('https://cdn.jsdelivr.net/gh/luoxuhai/pcl.js@master/data/rops_tutorial/points.pcd').then(res => res.arrayBuffer());
  // 写入 PCD 文件
  pcl.fs.writeFile('/test.pcd', pcd);
  // 加载 PCD 文件，返回点云对象
  const pointCloud = pcl.io.loadPCDFile('/test.pcd');

  // 使用 PassThrough 过滤器过滤点云
  // 参考: https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html#passthrough
  const pass = new pcl.filters.PassThrough();
  pass.setInputCloud(pointCloud);
  pass.setFilterFieldName('z');
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(pointCloud);

  // 将过滤后的点云对象保存为 PCD 文件
  pcl.io.savePCDFileASCII('/test-filtered.pcd', pointCloud);
  // 读取 PCD 文件内容， 内容为 ArrayBuffer
  const pcd = pcl.fs.readFile('/test-filtered.pcd');

  // 删除所有 PCD 文件
  pcl.fs.unlink('/test.pcd')
  pcl.fs.unlink('/test-filtered.pcd')
  // ...
}

main();
```

## 资源大小

> PCL.js Version: latest

| 资源          |                                                    链接                                                     |     大小      |
| :------------ | :---------------------------------------------------------------------------------------------------------: | :-----------: |
| pcl.js        |     [https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.js](https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.js)      | ~32.3k gzip’d |
| pcl-core.wasm | [https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm](https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.wasm) | ~198k gzip’d  |

## 模块

- [ ] features
- [x] filters 50%
- [ ] geometry
- [x] io 50%
- [ ] kdtree
- [ ] keypoints
- [ ] octree
- [ ] outofcore
- [ ] recognition
- [x] registration 10%
- [ ] sample_consensus
- [ ] search
- [ ] segmentation
- [ ] surface

## 支持的环境

- Chrome 57+
- Edge 16+
- Firefox 52+
- Safari 11+
- Opera 44+
- Safari on iOS 11+
- Chrome for Android 57+
- Node.js 11.0.0+
- Deno 1.0+


## 协议

[MIT](https://github.com/luoxuhai/pcl.js/blob/master/LICENSE)
