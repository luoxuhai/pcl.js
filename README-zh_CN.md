<p align="center">
  <h1 align="center">PCL.js</h1>
  <p align="center">用于浏览器的点云库 (<a href="https://github.com/PointCloudLibrary/pcl">PCL</a>)，由 WebAssembly 提供支持。</p>
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
  <a href="https://www.npmjs.com/package/pcl.js">
    <img src="https://img.shields.io/github/v/release/luoxuhai/pcl.js?label=latest" alt="Github Stable Release" />
  </a>
  <a href="https://www.npmtrends.com/pcl.js">
    <img src="https://img.shields.io/npm/dm/pcl.js" alt="Downloads" />
  </a>
  <a href="https://github.com/luoxuhai/pcl.js/stargazers">
    <img src="https://img.shields.io/github/stars/luoxuhai/pcl.js" alt="Github Stars" />
  </a>
</p>

<p align="center">
  <a href="./README.md">English</a> | 简体中文
</p>

## 资源

- [Documentation](https://pcljs.org/docs)
- [API Reference](https://pcljs.org/api)
- [Examples](https://pcljs.org/examples)

## 简介

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
    // 可选配置，自定义 WebAssembly 文件链接
    url: 'https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.wasm',
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
  // 初始化
  const pcl = await PCL.init({
    // 可选配置，自定义 WebAssembly 文件链接
    url: 'https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.wasm',
    // 也可以传人 WebAssembly 文件的 ArrayBuffer
    // arrayBuffer: ArrayBuffer
  });

  // ...
}

main();
</script>
```

## 模块

- [ ] features
- [ ] filters
- [ ] geometry
- [ ] io
- [ ] kdtree
- [ ] keypoints
- [ ] octree
- [ ] outofcore
- [ ] recognition
- [ ] registration
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
