
# SACSegmentation

`SACSegmentation` represents the Nodelet segmentation class for Sample Consensus methods and models, in the sense that it just creates a Nodelet wrapper for generic-purpose SAC-based segmentation.

More: <https://pointclouds.org/documentation/classpcl_1_1_s_a_c_segmentation.html>

## Example

```ts title="TypeScript" showLineNumbers
import * as PCl from 'pcl'

await PCL.init();

// data is noisy_slice_displaced.pcd: https://github.com/luoxuhai/pcl.js/tree/master/data/noisy_slice_displaced.pcd
const cloud = PCL.loadPCDData<PCL.PointXYZ>(data, PCL.PointXYZ);

// highlight-start
const coefficients = new PCL.ModelCoefficients();
const inliers = new PCL.PointIndices();
const seg = new PCL.SACSegmentation<PCL.PointXYZ>();

seg.setOptimizeCoefficients(true);
seg.setModelType(PCL.SacModelTypes.SACMODEL_SPHERE);
seg.setMethodType(PCL.SacMethodTypes.SAC_RANSAC);
seg.setMaxIterations(10000);
seg.setDistanceThreshold(0.01);
seg.setRadiusLimits(0.03, 0.07);
seg.setInputCloud(cloud);
seg.segment(inliers, coefficients);
// highlight-end
```

## Type Definitions

```ts
class SACSegmentation<T extends XYZPointTypes = PointXYZ>> extends PCLBase<T> {
  constructor(random?: boolean);
  setModelType(model: SacModelTypes): void;
  getModelType(): SacModelTypes;
  setMethodType(method: SacMethodTypes): void;
  getMethodType(): SacMethodTypes;
  setDistanceThreshold(threshold: number): void;
  getDistanceThreshold(): number;
  setMaxIterations(maxIterations: number): void;
  getMaxIterations(): number;
  setProbability(probability: number): void;
  getProbability(): number;
  setRadiusLimits(minRadius: number, maxRadius: number): void;
  getRadiusLimits(): {
      minRadius: number;
      maxRadius: number;
  };
  setSamplesMaxDist(radius: number, search: Search<T>): void;
  getSamplesMaxDist(): {
      radius: number;
      search: Search<T> | undefined;
  };
  setEpsAngle(epsAngle: number): void;
  getEpsAngle(): number;
  setOptimizeCoefficients(optimizeCoefficients: boolean): void;
  getOptimizeCoefficients(): boolean;
  segment(inliers: PointIndices, modelCoefficients: ModelCoefficients): void;
  /**
   * Only supports `PointXYZ` type,
   * if it is not `PointXYZ` type, it will use `toXYZPointCloud` method to convert to `PointXYZ`
   */
  setInputCloud(cloud: PointCloud<T>): void;
}
```
