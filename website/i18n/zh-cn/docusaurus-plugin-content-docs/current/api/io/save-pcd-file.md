---
sidebar_position: 3
---

# savePCDFile  

```ts
savePCDFile(filename, cloud, binaryMode)
```

将点云数据保存到包含特定给定云格式的 ASCII 或二进制 PCD 文件。

**参数:**

| 名称                  | 类型                                                | 默认值  | 描述                                      |
| --------------------- | --------------------------------------------------- | ------- | ----------------------------------------- |
| filename              | `string`                                            |         | 要加载的文件的名称。                      |
| cloud                 | [PointCloud](/docs/api/basic-structures#pointcloud) |         | 点云数据                                  |
| binaryMode (optional) | `boolean`                                           | `false` | `true` 表示二进制模式, `false` 表示 ASCII |


**返回:**

| 名称   | 类型      |
| ------ | --------- |
| status | `boolean` |