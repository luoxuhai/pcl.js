# IterativeClosestPoint

`IterativeClosestPoint` provides a base implementation of the Iterative Closest Point algorithm.

More: <https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html>

## Example

```ts title="TypeScript" showLineNumbers
await PCL.init();

// highlight-start
const icp = new PCL.IterativeClosestPoint<PCL.PointXYZ>();
icp.setInputSource(source);
icp.setInputTarget(target);
icp.setMaximumIterations(50);
icp.setTransformationEpsilon(1e-8);
icp.setMaxCorrespondenceDistance(0.05);
icp.align(final);
// highlight-end
```

## Type Definitions

### IterativeClosestPoint

```ts title="TypeScript" showLineNumbers
class IterativeClosestPoint<T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>> extends Registration<T> {
    constructor();
    setUseReciprocalCorrespondences(useReciprocalCorrespondence: boolean): void;
    getUseReciprocalCorrespondences(): boolean;
}
```

### Registration

See [Registration](/docs/api/registration/sample-consensus-initial-alignment#registration)
