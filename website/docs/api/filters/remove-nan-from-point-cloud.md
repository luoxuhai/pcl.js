# removeNaNFromPointCloud

Removes points with x, y, or z equal to NaN.

```ts
removeNaNFromPointCloud(cloudIn, cloudOut, indices)
```

## Type Definitions

```ts
function removeNaNFromPointCloud<T>(cloudIn: PointCloud<T>, cloudOut?: PointCloud<T>, indices?: Indices): {
    cloud: PointCloud<T>;
    indices: Indices;
};
```