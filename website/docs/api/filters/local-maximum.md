# LocalMaximum

> https://pointclouds.org/documentation/classpcl_1_1_local_maximum.html

`LocalMaximum` downsamples the cloud, by eliminating points that are locally maximal.

## Constructor

```ts
new pcl.filters.LocalMaximum(pointType, extractRemovedIndices);
```

**Parameters:**

| Name                  | Type                                              | Default    | Description                                                          |
| --------------------- | ------------------------------------------------- | ---------- | -------------------------------------------------------------------- |
| pointType             | [PointType](/docs/api/basic-structures#pointtype) | `PointXYZ` | The point cloud type.                                                |
| extractRemovedIndices | `boolean`                                         | `false`    | Initializing with true will allow us to extract the removed indices. |

## Methods

### setRadius

```ts
setRadius(radius);
```

Set the radius to use to determine if a point is the local max.

**Parameters:**

| Name   | Type     | Default | Description                                                 |
| ------ | -------- | ------- | ----------------------------------------------------------- |
| radius | `number` |         | The radius to use to determine if a point is the local max. |


### getRadius

```ts
getRadius();
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
