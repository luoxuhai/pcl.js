---
sidebar_position: 5
---

# savePCDFileBinary

```ts
savePCDFileBinary(filename, cloud)
```

Saving point cloud data to a binary PCD file containing a specific given cloud format.

**Parameters:**

| Name     | Type                                                | Default | Description                   |
| -------- | --------------------------------------------------- | ------- | ----------------------------- |
| filename | `string`                                            |         | The name of the file to load. |
| cloud    | [PointCloud](/docs/api/basic-structures#pointcloud) |         | The point cloud data message  |

**Returns:**

| Name   | Type      |
| ------ | --------- |
| status | `boolean` |