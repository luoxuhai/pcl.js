---
sidebar_position: 3
---

# savePCDFile  

```ts
savePCDFile(filename, cloud, binaryMode)
```

Saving point cloud data to a ASCII or binary PCD file containing a specific given cloud format.

**Parameters:**

| Name                  | Type                                                | Default | Description                               |
| --------------------- | --------------------------------------------------- | ------- | ----------------------------------------- |
| filename              | `string`                                            |         | The name of the file to load.             |
| cloud                 | [PointCloud](/docs/api/basic-structures#pointcloud) |         | The point cloud data message              |
| binaryMode (optional) | `boolean`                                           | `false` | `true` for binary mode, `false` for ASCII |


**Returns:**

| Name   | Type      |
| ------ | --------- |
| status | `boolean` |