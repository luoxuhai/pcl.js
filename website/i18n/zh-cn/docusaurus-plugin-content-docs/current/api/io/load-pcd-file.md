---
sidebar_position: 1
---

# loadPCDFile

```ts
loadPCDFile(filename, pointType)
```

将任何 PCD 文件加载到 `PointCloud` 类型中。

例如：

```ts
import * as PCL from "pcl.js"

await PCL.init()

// highlight-next-line
PCL.loadPCDFile("test.pcd", PCL.PointXYZ)
```

**参数:**

| 名称                 | 类型                                              | 默认值     | 描述                 |
| -------------------- | ------------------------------------------------- | ---------- | -------------------- |
| filename             | `string`                                          |            | 要加载的文件的名称。 |
| pointType (optional) | [PointType](/docs/api/basic-structures#pointtype) | `PointXYZ` | 生成的点云类型。     |

**返回:**

| 名称  | 类型                                                |
| ----- | --------------------------------------------------- |
| cloud | [PointCloud](/docs/api/basic-structures#pointcloud) |
