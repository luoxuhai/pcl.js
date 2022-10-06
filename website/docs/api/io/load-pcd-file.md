---
sidebar_position: 1
---

# loadPCDFile

```ts
loadPCDFile(filename, pointType)
```

Load any PCD file into a PointCloud type.

For example:

```ts
import * as PCL from "pcl.js"

await PCL.init()

// highlight-next-line
PCL.loadPCDFile("test.pcd", PCL.PointXYZ)
```

**Parameters:**

| Name                 | Type        | Default    | Description                     |
| -------------------- | ----------- | ---------- | ------------------------------- |
| filename             | `string`    |            | The name of the file to load.   |
| pointType (optional) | [PointType](/docs/api/basic-structures#pointtype) | `PointXYZ` | The resultant point cloud type. |

**Returns:**

| Name  | Type                                     |
| ----- | ---------------------------------------- |
| cloud | [PointCloud](/docs/api/basic-structures#pointcloud) |
