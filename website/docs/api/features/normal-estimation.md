# NormalEstimation

`NormalEstimation` estimates local surface properties (surface normals and curvatures)at each 3D point.

More: <https://pointclouds.org/documentation/classpcl_1_1_normal_estimation.html>

## Example

```ts title="TypeScript" showLineNumbers
await PCL.init();

// highlight-start
const tree = new PCL.SearchKdTree<PCL.PointXYZ>(PCL.PointXYZ);
const normals = new PCL.PointCloud<PCL.Normal>(PCL.Normal);

const normEst = new PCL.NormalEstimation();
normEst.setSearchMethod(tree);
normEst.setRadiusSearch(0.05);
normEst.setInputCloud(cloudTarget);
normEst.compute(normals);
// highlight-end
```

## Type Definitions

### NormalEstimation

```ts title="TypeScript" showLineNumbers
class NormalEstimation<PointInT extends XYZPointTypes, PointOutT extends Normal = Normal> extends Feature<PointInT, PointOutT> {
    constructor();
    setViewPoint(vpx: number, vpy: number, vpz: number): void;
    getViewPoint(): {
        vpx: number;
        vpy: number;
        vpz: number;
    };
    useSensorOriginAsViewPoint(): void;
}
```

### Feature

```ts title="TypeScript" showLineNumbers
class Feature<PointInT extends XYZPointTypes, PointOutT extends FPFHSignature33 | Normal = Normal> extends PCLBase<PointInT> {
    constructor(native: NativeAPI);
    setSearchSurface(cloud: PointCloud<PointInT>): void;
    setSearchMethod(searchMethod: Search<PointInT>): void;
    getSearchMethod(): Search<PointInT> | undefined;
    getSearchParameter(): number;
    setKSearch(k: number): void;
    getKSearch(): number;
    setRadiusSearch(radius: number): void;
    getRadiusSearch(): any;
    compute(cloud: PointCloud<PointOutT>): void;
}
```

### PCLBase

```ts title="TypeScript" showLineNumbers
class PCLBase<T extends PointTypes> {
    manager: Manager;
    constructor(_native: NativeAPI);
    setInputCloud(cloud: PointCloud<T>): void;
    getInputCloud(): PointCloud<T>;
}
```
