# RandomSample

> https://pointclouds.org/documentation/classpcl_1_1_random_sample.html

`RandomSample` applies a random sampling with uniform probability.

## Constructor

```ts
new PCL.RandomSample(pointType, extractRemovedIndices);
```

**Parameters:**

| Name                  | Type                                              | Default    | Description                                                          |
| --------------------- | ------------------------------------------------- | ---------- | -------------------------------------------------------------------- |
| pointType             | [PointType](/docs/api/basic-structures#pointtype) | `PointXYZ` | The point cloud type.                                                |
| extractRemovedIndices | `boolean`                                         | `false`    | Initializing with true will allow us to extract the removed indices. |

## Methods

### setSample

```ts
setSample(sample);
```

Set number of indices to be sampled.

**Parameters:**

| Name   | Type     | Default | Description                      |
| ------ | -------- | ------- | -------------------------------- |
| sample | `number` |         | Number of indices to be sampled. |

### getSample

```ts
getSample();
```

### setSeed

```ts
setSeed(seed);
```

Set seed of random function.

**Parameters:**

| Name | Type     | Default | Description              |
| ---- | -------- | ------- | ------------------------ |
| seed | `number` |         | seed of random function. |

### getSeed

```ts
getSeed();
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
