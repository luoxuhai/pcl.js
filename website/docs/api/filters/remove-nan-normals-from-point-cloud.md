# removeNaNNormalsFromPointCloud

Removes points that have their normals invalid (i.e., equal to NaN).

```ts
removeNaNNormalsFromPointCloud(cloudIn, cloudOut, indices)
```

## Type Definitions

```ts
function removeNaNNormalsFromPointCloud<T extends Normal | PointNormal>(cloudIn: PointCloud<T>, cloudOut?: PointCloud<T>, indices?: Indices): {
    cloud: PointCloud<T>;
    indices: Indices;
};
```