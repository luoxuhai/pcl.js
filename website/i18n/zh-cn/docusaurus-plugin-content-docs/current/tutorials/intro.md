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

  // 获取 PCD 文件
  const data = await fetch('https://cdn.jsdelivr.net/gh/luoxuhai/pcl.js@master/data/rops_tutorial/points.pcd').then(res => res.arrayBuffer());
  // 加载PCD文件数据，返回点云对象
  const cloud = PCL.loadPCDData<PCL.PointXYZ>(data, PCL.PointXYZ);

  // 使用 StatisticalOutlierRemoval 过滤器去除异常值
  // 参考: https://pcl.readthedocs.io/projects/tutorials/en/master/statistical_outlier.html#statistical-outlier-removal
  // highlight-start
  const sor = new PCL.StatisticalOutlierRemoval<PCL.PointXYZ>(PCL.PointXYZ);
  sor.setInputCloud(cloud);
  sor.setMeanK(40);
  sor.setStddevMulThresh(1.0);
  const cloudFiltered = sor.filter();
  // highlight-end

  // 将过滤后的点云对象保存为PCD文件，内容为ArrayBuffer
  const cloudFilteredData = PCL.savePCDDataASCII(cloudFiltered);
}

main();
```

|          过滤前        |          过滤后           |
| :--------------------------------: | :--------------------------: |
| ![JavaScript](/img/intro-1.jpg) | ![C++](/img/intro-2.jpg) |

[![Edit pcl.js-StatisticalOutlierRemoval](https://codesandbox.io/static/img/play-codesandbox.svg)](https://codesandbox.io/s/pcl-js-statisticaloutlierremoval-kl2zjs?fontsize=14&hidenavigation=1&theme=dark)
