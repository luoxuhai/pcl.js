---
sidebar_position: 5
---

# savePCDFileBinary

```ts
savePCDFileBinary(filename, cloud)
```

将点云数据保存到包含特定给定云格式的二进制 PCD 文件中。

**参数:**

| 名称     | 类型                                                | 默认值 | 描述                 |
| -------- | --------------------------------------------------- | ------ | -------------------- |
| filename | `string`                                            |        | 要加载的文件的名称。 |
| cloud    | [PointCloud](/docs/api/basic-structures#pointcloud) |        | 点云数据             |

**返回:**

| 名称   | 类型      |
| ------ | --------- |
| status | `boolean` |