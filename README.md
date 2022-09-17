<p align="center">
  <a href="https://pcljs.org" target="_blank"><img style="max-height: 100px" src="./logo.svg" title="pcl.js" alt="title="pcl.js"></a>
  <p align="center"><a href="https://github.com/PointCloudLibrary/pcl" target="_blank">Point Cloud Library (PCL)</a> for browser, powered by WebAssembly.</p>
</p>

<p align="center">
 <a href="https://github.com/FoalTS/foal/blob/master/LICENSE">
    <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT">
  </a>
 <a href="https://bundlephobia.com/package/pcl.js">
    <img src="https://img.shields.io/bundlephobia/min/pcl.js.svg" alt="Bundle Size">
 </a>
  <a href="https://www.npmjs.com/package/pcl.js">
    <img src="https://img.shields.io/npm/v/pcl.js.svg" alt="npm version">
  </a>
  <a href="https://www.npmtrends.com/pcl.js">
    <img src="https://img.shields.io/npm/dm/pcl.js" alt="Downloads" />
  </a>
  <a href="https://www.jsdelivr.com/package/npm/pcl.js">
    <img src="https://data.jsdelivr.com/v1/package/npm/pcl.js/badge?style=rounded" alt="jsDelivr" />
  </a>
  <a href="https://github.com/luoxuhai/pcl.js/actions/workflows/test.yml">
    <img src="https://github.com/luoxuhai/pcl.js/actions/workflows/test.yml/badge.svg" alt="Tests" />
  </a>
  <a href="https://openbase.com/js/pcl.js?utm_source=embedded&amp;utm_medium=badge&amp;utm_campaign=rate-badge">
    <img src="https://badges.openbase.com/js/rating/pcl.js.svg?token=nF4Z9XUsUhOe5yeVDZTPwpdoKqqamFbVBoVA5zbU5iM=" alt="Rate this package" />
  </a>
  <a href="https://deepscan.io/dashboard#view=project&tid=18815&pid=22098&bid=649724">
    <img alt="DeepScan" src="https://deepscan.io/api/teams/18815/projects/22098/branches/649724/badge/grade.svg">
  </a>
</p>

<p align="center">
  English | <a href="./README-zh_CN.md">简体中文</a>
</p>

## Overview

**pcl.js** is a [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl) that runs in the browser, powered by [Emscripten](https://emscripten.org/index.html) and [WebAssembly](https://webassembly.org/). [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl) is a standalone, large scale, open project for 2D/3D image and point cloud processing. 

**Removing outliers from point cloud data using a [StatisticalOutlierRemoval](https://pcl.readthedocs.io/projects/tutorials/en/master/statistical_outlier.html#statistical-outlier-removal) filter demo**
<p align="center">
  <a href="https://pcljs.org/examples">
    <img src="./website/static/img/examples/StatisticalOutlierRemoval.gif">
  </a>
</p>

## Features

- Provides an API similar to PCL(C++), easy to use
- Supports all modern browsers, will improve Nodejs support
- Written in TypeScript, with predictable static typing
- And many, many more! 🚀

## Resources

- [Documentation](https://pcljs.org/docs/tutorials/intro)
- [API Reference](https://pcljs.org/docs/api/about)
- [Examples](https://pcljs.org/examples)
- [Discussions](https://github.com/luoxuhai/pcl.js/discussions)

## Environment Support

| <img src="https://raw.githubusercontent.com/alrra/browser-logos/main/src/edge/edge_128x128.png" alt="Edge" width="48px" height="48px" /><br/> Edge | <img src="https://raw.githubusercontent.com/alrra/browser-logos/main/src/firefox/firefox_128x128.png" alt="Firefox" width="48px" height="48px" /><br/>Firefox | <img src="https://raw.githubusercontent.com/alrra/browser-logos/main/src/chrome/chrome_128x128.png" alt="Chrome" width="48px" height="48px" /><br/>Chrome | <img src="https://raw.githubusercontent.com/alrra/browser-logos/main/src/safari/safari_128x128.png" alt="Safari" width="48px" height="48px" /><br/>Safari | <img src="https://raw.githubusercontent.com/alrra/browser-logos/main/src/opera/opera_128x128.png" alt="Opera" width="48px" height="48px" /><br/>Opera |
| --------- | --------- | --------- | --------- | --------- |
| 16+ | 52+ | 57+ | 11+ | 44+ 

## Bundle Size

> pcl.js version: latest

| Source        |                                                    Link                                                     |     Size      |
| :------------ | :---------------------------------------------------------------------------------------------------------: | :-----------: |
| pcl.js        |     [https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.js](https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.js)      | ~34k gzip’d |
| pcl-core.wasm | [https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm](https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm) | ~360k gzip’d  |

## Installation

### NPM

```bash
# NPM
npm install pcl.js

# Yarn
yarn add pcl.js
```

### CDN

```html
<!-- Development -->
<script src="https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.js"><script>

<!-- Production -->
<script src="https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl.min.js"><script>
```

## Usage

### NPM

```typescript
import * as PCL from 'pcl.js';

async function main() {
  // Initialization
  const pcl = await PCL.init({
    /**
     * Recommend, optional configuration, custom WebAssembly file link.
     * @default js file dir + pcl-core.wasm
     */
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
  const pcl = await PCL.init();
  // ...
}

main();
</script>
```

### Basic Usage Example

```typescript
import * as PCL from 'pcl.js';

async function main() {
  const pcl = await PCL.init({
    url: 'https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm',
  });

  // Get PCD file
  const pcd = await fetch('https://cdn.jsdelivr.net/gh/luoxuhai/pcl.js@master/data/rops_tutorial/points.pcd').then(res => res.arrayBuffer());
  // Write a PCD file
  pcl.fs.writeFile('/test.pcd', new Uint8Array(pcd));
  // Load PCD file, return point cloud object
  const cloud = pcl.io.loadPCDFile<PCL.PointXYZ>('/test.pcd', PCL.PointXYZ);

  // Filtering a PointCloud using a PassThrough filter
  // See: https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html#passthrough
  const pass = new pcl.filters.PassThrough<PCL.PointXYZ>(PCL.PointXYZ);
  pass.setInputCloud(cloud);
  pass.setFilterFieldName('z');
  pass.setFilterLimits(0.0, 1.0);
  const cloudFiltered = pass.filter();
  // It can also be saved in the same way as in C++:
  // const cloudFiltered = pcl.common.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);
  // pass.filter(cloudFiltered);

  // Save filtered point cloud objects as PCD files
  pcl.io.savePCDFileASCII('/test-filtered.pcd', cloudFiltered);
  // Read PCD file content, the content is ArrayBuffer
  const pcd = pcl.fs.readFile('/test-filtered.pcd');

  // Delete all PCD files
  pcl.fs.unlink('/test.pcd')
  pcl.fs.unlink('/test-filtered.pcd')
  // ...
}

main();
```

## Project Layout

- [`src`](/src) All the source code for pcl.js, if you want to edit a api or just see how it works this is where you'll find it.
- [`website`](/website) The source code for https://pcljs.org
- [`tests`](/tests) You'll do most of your testing in here.

## Changelog

The [changelog](https://github.com/luoxuhai/pcl.js/releases) is regularly updated to reflect what's changed in each new release.

## Roadmap

Checkout the full roadmap [here](ROADMAP.md).

## Contributing

pcl.js has adopted the [Contributor Covenant](https://www.contributor-covenant.org/) as its Code of Conduct, and we expect project participants to adhere to it. Please read [the full text](CODE_OF_CONDUCT.md) so that you can understand what actions will and will not be tolerated.

Please make sure to read the [Contributing Guide](CONTRIBUTING.md) before making a pull request.

Thank you to all the people who already contributed to pcl.js!

[![Contributors](https://contrib.rocks/image?repo=luoxuhai/pcl.js)](https://github.com/luoxuhai/pcl.js/graphs/contributors)

## License

This project is licensed under the terms of the [MIT license](https://github.com/luoxuhai/pcl.js/blob/master/LICENSE).
