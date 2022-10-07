---
sidebar_position: 1
---

# 介绍

**pcl.js** 是在浏览器中运行的 [Point Cloud Library (PCL)](https://pointclouds.org/)，由 [Emscripten](https://emscripten.org/index.html) 和 [WebAssembly](https://webassembly.org/) 提供支持。

**PCL（Point Cloud Library）** 是在吸收了前人点云相关研究基础上建立起来的大型跨平台开源 C++ 编程库，它实现了大量点云相关的通用算法和高效数据结构，涉及到点云**获取、滤波、分割、配准、检索、特征提取、识别、追踪、曲面重建、可视化**等。支持多种操作系统平台，可在Windows、Linux、Android、Mac OS X、部分嵌入式实时系统上运行。如果说 OpenCV 是 2D 信息获取与处理的结晶，那么 PCL 就在 3D 信息获取与处理上具有同等地位，PCL 是 BSD 授权方式，可以免费进行商业和学术应用。

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
