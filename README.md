<p align="center">
  <h1 align="center" >PCL.js</h1>
  <p align="center">Point Cloud Library (<a href="https://github.com/PointCloudLibrary/pcl" target="_blank">PCL</a>) for browser, powered by WebAssembly.</p>
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
</p>

<p align="center">
  English | <a href="./README-zh_CN.md">简体中文</a>
</p>

## Resources

- [Documentation](https://pcljs.org/docs)
- [API Reference](https://pcljs.org/api)
- [Examples](https://pcljs.org/examples)

## Overview

## Installation

### NPM

```bash
npm install pcl.js

or

yarn add pcl.js
```

### CDN

#### Development

```html
<script src="https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.js"><script>
```

#### Production

```html
<script src="https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.min.js"><script>
```

## Usage

### NPM

```typescript
import PCL from 'pcl.js';

async function main() {
  // Initialization
  const pcl = await PCL.init({
    // Recommend, optional configuration, custom WebAssembly file link.
    url: 'https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm',
    // You can also pass an ArrayBuffer of WebAssembly files.
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
  // Initialization, PCL is a global object.
  const pcl = await PCL.init({
    // Recommend, optional configuration, custom WebAssembly file link.
    url: 'https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm',
    // You can also pass an ArrayBuffer of WebAssembly files.
    // arrayBuffer: ArrayBuffer
  });

  // ...
}

main();
</script>
```

## Bundle Size

> PCL.js Version: 0.2.0  
> PCL Version: [1.12.1](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.12.1)

| Source        |                                                    Link                                                     |     Size      |
| :------------ | :---------------------------------------------------------------------------------------------------------: | :-----------: |
| pcl.js        |     [https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.js](https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.js)      | ~32.3k gzip’d |
| pcl.min.js    | [https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.min.js](https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.min.js)  | ~23.5k gzip’d |
| pcl-core.wasm | [https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm](https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.wasm) | ~198k gzip’d  |

## Modules

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

## Environment Support

- Chrome 57+
- Edge 16+
- Firefox 52+
- Safari 11+
- Opera 44+
- Safari on iOS 11+
- Chrome for Android 57+
- Node.js 11.0.0+
- Deno 1.0+

## License

[MIT](https://github.com/luoxuhai/pcl.js/blob/master/LICENSE)
