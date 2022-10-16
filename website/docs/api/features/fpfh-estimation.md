# FPFHEstimation

`FPFHEstimation` estimates the Fast Point Feature Histogram (FPFH) descriptor for a given point cloud dataset containing points and normals.

:::tip TIP

A commonly used type for PointOutT is [`FPFHSignature33`](/docs/api/basic-structures#fpfhsignature33).

:::

More: <https://pointclouds.org/documentation/classpcl_1_1_f_p_f_h_estimation.html>

## Example

```ts title="TypeScript" showLineNumbers
await PCL.init();

// highlight-start
const featuresSource = new PCL.PointCloud<PCL.FPFHSignature33>(PCL.FPFHSignature33);
const featuresTarget = new PCL.PointCloud<PCL.FPFHSignature33>(PCL.FPFHSignature33);
const fpfhEst = new PCL.FPFHEstimation();
fpfhEst.setSearchMethod(tree);
fpfhEst.setRadiusSearch(0.05);
fpfhEst.setInputCloud(cloudSource);
fpfhEst.setInputNormals(normals);
fpfhEst.compute(featuresSource);
// highlight-end
```

## Type Definitions

### FPFHEstimation

```ts title="TypeScript" showLineNumbers
class FPFHEstimation<PointInT extends XYZPointTypes, PointNT extends Normal = Normal, PointOutT extends FPFHSignature33 = FPFHSignature33> extends FeatureFromNormals<PointInT, PointNT, PointOutT> {
    constructor();
    setNrSubdivisions(nrBinsF1: number, nrBinsF2: number, nrBinsF3: number): void;
    getNrSubdivisions(): {
        nrBinsF1: number;
        nrBinsF2: number;
        nrBinsF3: number;
    };
}
```

### FeatureFromNormals

```ts title="TypeScript" showLineNumbers
class FeatureFromNormals<PointInT extends XYZPointTypes, PointNT extends Normal = Normal, PointOutT extends FPFHSignature33 = FPFHSignature33> extends Feature<PointInT, PointOutT> {
    constructor(native: NativeAPI);
    setInputNormals(normals: PointCloud<PointNT>): void;
    getInputNormals(): PointCloud<PointNT> | undefined;
}
```

### Feature

See [Feature](/docs/api/features/normal-estimation#Feature)
