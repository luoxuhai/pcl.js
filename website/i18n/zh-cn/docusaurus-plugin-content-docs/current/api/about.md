---
sidebar_position: 1
---

# 关于

pcl.js 的 api 必须在初始化成功后调用，例如：

```ts showLineNumbers
import * as PCL from 'pcl.js'

// highlight-next-line
await PCL.init();

// 调用 api
new PCL.PointCloud();
```
