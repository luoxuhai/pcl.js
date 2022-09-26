---
sidebar_position: 1
---

# About

The api of pcl.js(except [Visualization](/docs/category/visualization)) must be called after successful initialization, for Example:

```ts showLineNumbers
import * as PCL from 'pcl.js'

// highlight-next-line
const pcl = await PCL.init();

// call api
pcl.io.PointCloud();
```