
# MinCutSegmentation

This class implements the segmentation algorithm based on minimal cut of the graph.

More:

1. <https://pointclouds.org/documentation/classpcl_1_1_min_cut_segmentation.html>
2. <https://pcl.readthedocs.io/projects/tutorials/en/master/min_cut_segmentation.html#min-cut-segmentation>

## Example

```ts title="TypeScript" showLineNumbers
import * as PCl from 'pcl'

await PCL.init();

// highlight-start
const mcSeg = new PCL.MinCutSegmentation<PCL.PointXYZ>(
  PCL.PointXYZ,
);
const objectCenter = new PCL.PointXYZ(68.97, -18.55, 0.57);
const foregroundPoints = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);
foregroundPoints.addPoint(objectCenter);

mcSeg.setForegroundPoints(foregroundPoints);
mcSeg.setInputCloud(cloud);
mcSeg.setRadius(3.0433856);
mcSeg.setSigma(0.25);
mcSeg.setSourceWeight(0.8);
mcSeg.setNumberOfNeighbours(14);
const clusters = mcSeg.extract();
// highlight-end
```

## Type Definitions

```ts
class MinCutSegmentation<T> {
    _native: Emscripten.NativeAPI;
    constructor(PT?: XYZPointTypesTypeof);
    setInputCloud(cloud: PointCloud<T>): any;
    setSigma(sigma: number): void;
    getSigma(): number;
    setRadius(radius: number): void;
    getRadius(): number;
    setSourceWeight(weight: number): void;
    getSourceWeight(): number;
    setSearchMethod(tree: string | null): void;
    getSearchMethod(): string | null;
    setNumberOfNeighbours(neighbourNumber: number): void;
    getNumberOfNeighbours(): number;
    setForegroundPoints(cloud: PointCloud<T>): any;
    getForegroundPoints(): PointCloud<T>;
    setBackgroundPoints(cloud: PointCloud<T>): any;
    getBackgroundPoints(): PointCloud<T>;
    extract(): Vector<unknown>;
    getMaxFlow(): number;
    getColoredCloud(): PointCloud<T>;
}
```
