---
sidebar_position: 1
---

# Introduction

**pcl.js** is a [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl) that runs in the browser, powered by [Emscripten](https://emscripten.org/index.html) and [WebAssembly](https://webassembly.org/). [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl) is a standalone, large scale, open project for 2D/3D image and point cloud processing.

**pcl.js** uses the same API as PCL, is optimized for JavaScript, and provides TypeScript types, which are easy to use.

## Basic Usage Example

```typescript showLineNumbers title=TypeScript
import * as PCL from 'pcl.js';

async function main() {
  // Initialization
  await PCL.init({
    url: 'https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm',
  });

  // Get PCD file
  const data = await fetch('https://cdn.jsdelivr.net/gh/luoxuhai/pcl.js@master/data/table_scene_lms400.pcd').then(res => res.arrayBuffer());
  // Load PCD file data, return point cloud object
  const cloud = PCL.loadPCDData<PCL.PointXYZ>(data, PCL.PointXYZ);

  // Filtering a PointCloud using a PassThrough filter
  // See: https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html#passthrough
  // highlight-start
  const sor = new PCL.StatisticalOutlierRemoval<PCL.PointXYZ>(PCL.PointXYZ);
  sor.setInputCloud(cloud);
  sor.setMeanK(40);
  sor.setStddevMulThresh(1.0);
  const cloudFiltered = sor.filter();
  // highlight-end
  // Save filtered point cloud objects as PCD files, the content is ArrayBuffer
  const cloudFilteredData = PCL.savePCDDataASCII(cloudFiltered);
}

main();
```

|         Original         |          Filtered           |
| :--------------------------------: | :--------------------------: |
| ![JavaScript](/img/intro-1.jpg) | ![C++](/img/intro-2.jpg) |

[![Edit pcl.js-StatisticalOutlierRemoval](https://codesandbox.io/static/img/play-codesandbox.svg)](https://codesandbox.io/s/pcl-js-statisticaloutlierremoval-kl2zjs?fontsize=14&hidenavigation=1&theme=dark)
