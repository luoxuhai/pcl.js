---
sidebar_position: 1
---

# 介绍

**pcl.js** 是在浏览器中运行的 [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl)，由 [Emscripten](https://emscripten.org/index.html) 和 [WebAssembly](https://webassembly.org/) 提供支持。[Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl) 是一个独立、大型、开源的 2D/3D 图像和点云处理库。

**pcl.js** 在使用了与 PCL 相同的 API，并针对 JavaScript 进行优化，提供了 TypeScript 类型，使用方便。同一段逻辑在 Javascript 和 C++ 中的写法如下如下所示：

|         pcl.js(JavaScript)         |          PCL(C++ )           |
| :--------------------------------: | :--------------------------: |
| ![JavaScript](/img/js-example.png) | ![C++](/img/c++-example.png) |
