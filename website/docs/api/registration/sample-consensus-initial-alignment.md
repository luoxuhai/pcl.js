# SampleConsensusInitialAlignment

`SampleConsensusInitialAlignment` is an implementation of the initial alignment algorithm described in section IV of "Fast Point Feature Histograms (FPFH) for 3D Registration," Rusu et al.

More: <https://pointclouds.org/documentation/classpcl_1_1_sample_consensus_initial_alignment.html>

## Example

```ts title="TypeScript" showLineNumbers
await PCL.init();

// highlight-start
const reg = new PCL.SampleConsensusInitialAlignment();
reg.setMinSampleDistance(0.05);
reg.setMaxCorrespondenceDistance(0.1);
reg.setMaximumIterations(1000);
reg.setInputSource(cloudSource);
reg.setInputTarget(cloudTarget);
reg.setSourceFeatures(featuresSource);
reg.setTargetFeatures(featuresTarget);
reg.align(final);
// highlight-end
```

## Type Definitions

### SampleConsensusInitialAlignment

```ts title="TypeScript" showLineNumbers
class SampleConsensusInitialAlignment<T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>, FeatureT extends FPFHSignature33 = FPFHSignature33> extends Registration<T> {
    constructor();
    setSourceFeatures(cloud: PointCloud<FeatureT>): void;
    getSourceFeatures(): PointCloud<FeatureT>;
    setTargetFeatures(cloud: PointCloud<FeatureT>): void;
    getTargetFeatures(): PointCloud<FeatureT>;
    setMinSampleDistance(minSampleDistance: number): void;
    getMinSampleDistance(): number;
    setNumberOfSamples(samples: number): void;
    getNumberOfSamples(): number;
    setCorrespondenceRandomness(k: number): void;
    getCorrespondenceRandomness(): number;
}
```

### Registration

```ts title="TypeScript" showLineNumbers
class Registration<T extends XYZPointTypes> extends PCLBase<T> {
    constructor(native: NativeAPI);
    /**
     * Only supports `PointXYZ` type,
     * if it is not `PointXYZ` type, it will use `toXYZPointCloud` method to convert to `PointXYZ`
     */
    setInputSource(cloud: PointCloud<T>): void;
    getInputSource(): PointCloud<PointXYZ>;
    /**
     * Only supports `PointXYZ` type,
     * if it is not `PointXYZ` type, it will use `toXYZPointCloud` method to convert to `PointXYZ`
     */
    setInputTarget(cloud: PointCloud<T>): void;
    getInputTarget(): PointCloud<PointXYZ>;
    getFitnessScore(maxRange?: number): number;
    hasConverged(): boolean;
    setMaximumIterations(maxIterations: number): void;
    getMaximumIterations(): number;
    setRANSACIterations(ransacIterations: number): void;
    getRANSACIterations(): number;
    setRANSACOutlierRejectionThreshold(inlierThreshold: number): void;
    getRANSACOutlierRejectionThreshold(): number;
    setMaxCorrespondenceDistance(distanceThreshold: number): void;
    getMaxCorrespondenceDistance(): number;
    setTransformationEpsilon(epsilon: number): void;
    getTransformationEpsilon(): number;
    setTransformationRotationEpsilon(epsilon: number): void;
    getTransformationRotationEpsilon(): number;
    setEuclideanFitnessEpsilon(epsilon: number): void;
    getEuclideanFitnessEpsilon(): number;
    initCompute(): boolean;
    initComputeReciprocal(): boolean;
    align(output: PointCloud<T>): void;
}
```

### PCLBase

See [PCLBase](/docs/api/features/normal-estimation#PCLBase)
