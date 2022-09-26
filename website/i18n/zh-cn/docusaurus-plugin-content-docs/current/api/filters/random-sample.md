# RandomSample

> https://pointclouds.org/documentation/classpcl_1_1_random_sample.html

`RandomSample` 应用具有均匀概率的随机抽样。

## 构造函数

```ts
new pcl.filters.RandomSample(pointType, extractRemovedIndices);
```

**参数:**

| 名称                  | 类型                                              | 默认值     | 描述                                       |
| --------------------- | ------------------------------------------------- | ---------- | ------------------------------------------ |
| pointType             | [PointType](/docs/api/basic-structures#pointtype) | `PointXYZ` | 点云类型。                                 |
| extractRemovedIndices | `boolean`                                         | `false`    | 使用 true 初始化将允许我们提取删除的索引。 |

## 方法

### setSample

```ts
setSample(sample);
```

设置要采样的索引数。

**参数:**

| 名称   | 类型     | 默认值 | 描述             |
| ------ | -------- | ------ | ---------------- |
| sample | `number` |        | 要采样的索引数。 |

### getSample

```ts
getSample();
```

### setSeed

```ts
setSeed(seed);
```

设置随机函数的种子。

**参数:**

| 名称 | 类型     | 默认值 | 描述             |
| ---- | -------- | ------ | ---------------- |
| seed | `number` |        | 随机函数的种子。 |

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
