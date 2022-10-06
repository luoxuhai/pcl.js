---
sidebar_position: 1
---

# loadPCDData

```ts
loadPCDData(data, pointType)
```

Load any PCD data into a PointCloud type.

For example:

```ts
import * as PCL from "pcl.js"

await PCL.init()

// highlight-next-line
PCL.loadPCDData(ArrayBuffer)
```

**Parameters:**

| Name                 | Type        | Default    | Description                     |
| -------------------- | ----------- | ---------- | ------------------------------- |
| data             | `ArrayBuffer`    |            | The PCD file data.   |
| pointType (optional) | [PointType](/docs/api/basic-structures#pointtype) | `PointXYZ` | The resultant point cloud type. |

**Returns:**

| Name  | Type                                     |
| ----- | ---------------------------------------- |
| cloud | [PointCloud](/docs/api/basic-structures#pointcloud) |
