# ApproximateVoxelGrid

> https://pointclouds.org/documentation/classpcl_1_1_approximate_voxel_grid.html

`ApproximateVoxelGrid` assembles a local 3D grid over a given `PointCloud`, and downsamples + filters the data.

## Constructor

```ts
new pcl.filters.ApproximateVoxelGrid(pointType);
```

**Parameters:**

| Name      | Type                                              | Default    | Description           |
| --------- | ------------------------------------------------- | ---------- | --------------------- |
| pointType | [PointType](/docs/api/basic-structures#pointtype) | `PointXYZ` | The point cloud type. |

## Methods

### setLeafSize()

```ts
setLeafSize(lx, ly, lz)
```

**Parameters:**

| Name | Type     | Default | Description          |
| ---- | -------- | ------- | -------------------- |
| lx   | `number` |         | the leaf size for X. |
| ly   | `number` |         | the leaf size for Y. |
| lz   | `number` |         | the leaf size for Z. |

### setDownsampleAllData()

```ts
setLeafSize(downsample);
```

Set to true if all fields need to be downsampled, or false if just XYZ.

**Parameters:**

| Name       | Type      | Default | Description                |
| ---------- | --------- | ------- | -------------------------- |
| downsample | `boolean` |         | The new value (true/false) |
| .          |

### getDownsampleAllData()

```ts
getDownsampleAllData();
```

Get the state of the internal downsampling parameter (true if all fields need to be downsampled, false if just XYZ).

**Returns:**

| Name       | Type      |
| ---------- | --------- |
| downsample | `boolean` |

### setInputCloud

See [PassThrough.setInputCloud](/docs/api/filters/pass-through#setinputcloud)

### getInputCloud

See [PassThrough.getInputCloud](/docs/api/filters/pass-through#getinputcloud)

### filter

See [PassThrough.filter](/docs/api/filters/pass-through#filter)
