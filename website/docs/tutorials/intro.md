---
sidebar_position: 1
---

# Introduction

**pcl.js** is a [Point Cloud Library (PCL)](https://pointclouds.org/) that runs in the browser, powered by [Emscripten](https://emscripten.org/index.html) and [WebAssembly](https://webassembly.org/).

**The Point Cloud Library (or PCL)** is a large scale, open project for 2D/3D image and point cloud processing. The PCL framework contains numerous state-of-the art algorithms including **filtering, feature estimation, surface reconstruction, registration, model fitting and segmentation**. These algorithms can be used, for example, to filter outliers from noisy data, stitch 3D point clouds together, segment relevant parts of a scene, extract keypoints and compute descriptors to recognize objects in the world based on their geometric appearance, and create surfaces from point clouds and visualize them – to name a few.

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

  // Removing outliers using a StatisticalOutlierRemoval filter
  // See: https://pcl.readthedocs.io/projects/tutorials/en/master/statistical_outlier.html#statistical-outlier-removal
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
