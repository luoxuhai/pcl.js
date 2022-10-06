---
sidebar_position: 1
---

# 介绍

**pcl.js** 是在浏览器中运行的 [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl)，由 [Emscripten](https://emscripten.org/index.html) 和 [WebAssembly](https://webassembly.org/) 提供支持。[Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl) 是一个独立、大型、开源的 2D/3D 图像和点云处理库。

**pcl.js** 在使用了与 PCL 相同的 API，并针对 JavaScript 进行优化，提供了 TypeScript 类型，使用方便。

## 简单示例

```typescript showLineNumbers title=TypeScript
import * as PCL from 'pcl.js';

async function main() {
  await PCL.init({
    url: 'https://cdn.jsdelivr.net/npm/pcl.js/dist/pcl-core.wasm',
  });

  // Get PCD file
  const data = await fetch('https://cdn.jsdelivr.net/gh/luoxuhai/pcl.js@master/data/rops_tutorial/points.pcd').then(res => res.arrayBuffer());
  // Load PCD file data, return point cloud object
  const cloud = PCL.loadPCDData<PCL.PointXYZ>(data, PCL.PointXYZ);

  // Filtering a PointCloud using a PassThrough filter
  // See: https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html#passthrough
  // highlight-start
  const pass = new PCL.PassThrough<PCL.PointXYZ>(PCL.PointXYZ);
  pass.setInputCloud(cloud);
  pass.setFilterFieldName('z');
  pass.setFilterLimits(0.0, 1.0);
  const cloudFiltered = pass.filter();
  // highlight-end

  // Save filtered point cloud objects as PCD files, the content is ArrayBuffer
  const cloudFilteredData = PCL.savePCDDataASCII(cloudFiltered);
}

main();
```

|          过滤前        |          过滤后           |
| :--------------------------------: | :--------------------------: |
| ![JavaScript](/img/intro-1.jpg) | ![C++](/img/intro-2.jpg) |

[![Edit pcl.js-StatisticalOutlierRemoval](https://codesandbox.io/static/img/play-codesandbox.svg)](https://codesandbox.io/s/pcl-js-statisticaloutlierremoval-kl2zjs?fontsize=14&hidenavigation=1&theme=dark)
