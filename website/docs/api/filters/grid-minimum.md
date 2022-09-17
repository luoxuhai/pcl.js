# GridMinimum

> https://pointclouds.org/documentation/classpcl_1_1_grid_minimum.html

`GridMinimum` assembles a local 2D grid over a given [`PointCloud`](/docs/api/basic-structures#pointcloud), and downsamples the data.

## Constructor

```ts
new pcl.filters.GridMinimum(pointType, resolution);
```

**Parameters:**

| Name       | Type                                              | Default    | Description           |
| ---------- | ------------------------------------------------- | ---------- | --------------------- |
| pointType  | [PointType](/docs/api/basic-structures#pointtype) | `PointXYZ` | The point cloud type. |
| resolution | `number`                                          | `0`        | The grid resolution.  |

## Methods

### setResolution

```ts
setResolution(resolution);
```

**Parameters:**

| Name       | Type     | Default | Description          |
| ---------- | -------- | ------- | -------------------- |
| resolution | `number` |         | The grid resolution. |


### getResolution

```ts
getResolution();
```

### setNegative

See [PassThrough.setNegative](/docs/api/filters/pass-through#setnegative)

### getNegative

See [PassThrough.getNegative](/docs/api/filters/pass-through#getnegative)

### setKeepOrganized

See [PassThrough.setKeepOrganized](/docs/api/filters/pass-through#setkeeporganized)

### getKeepOrganized

See [PassThrough.getKeepOrganized](/docs/api/filters/pass-through#getkeeporganized)

### setUserFilterValue

See [PassThrough.setUserFilterValue](/docs/api/filters/pass-through#setuserfiltervalue)

### setInputCloud

See [PassThrough.setInputCloud](/docs/api/filters/pass-through#setinputcloud)

### getInputCloud

See [PassThrough.getInputCloud](/docs/api/filters/pass-through#getinputcloud)

### filter

See [PassThrough.filter](/docs/api/filters/pass-through#filter)
