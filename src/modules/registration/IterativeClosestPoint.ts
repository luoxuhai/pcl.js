import {
  PointCloud,
  createPointCloud,
  PointTypes,
  PointTypesMerge,
  PointTypesUnion,
} from '../point-types';

class IterativeClosestPoint<
  T extends Partial<PointTypesUnion> = Partial<PointTypesMerge>,
> {
  public native: Emscripten.NativeAPI;

  constructor(pointType = PointTypes.PointXYZ) {
    this.native = new __PCLCore__[`IterativeClosestPoint${pointType}`]();
  }

  public setInputSource(cloud: PointCloud<T>) {
    return this.native.setInputSource(cloud.native);
  }

  public setInputTarget(cloud: PointCloud<T>) {
    return this.native.setInputTarget(cloud.native);
  }

  public getFinalTransformation(): string | null {
    return this.native.getFinalTransformation() as string | null;
  }

  public getFitnessScore(): number {
    return this.native.getFitnessScore();
  }

  public hasConverged() {
    return this.native.hasConverged();
  }

  public setUseReciprocalCorrespondences(useReciprocalCorrespondence: boolean) {
    return this.native.setUseReciprocalCorrespondences(
      useReciprocalCorrespondence,
    );
  }

  public getUseReciprocalCorrespondences(): boolean {
    return this.native.getUseReciprocalCorrespondences();
  }

  public align() {
    return createPointCloud<T>(this.native.align());
  }
}

export default IterativeClosestPoint;
