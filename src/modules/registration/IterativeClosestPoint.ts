import {
  PointCloud,
  wrapPointCloud,
  PointXYZ,
  PointTypesIntersection,
  PointTypesUnion,
  TPointTypesUnion,
} from '../point-types';

class IterativeClosestPoint<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> {
  public native: Emscripten.NativeAPI;

  constructor(_PT: TPointTypesUnion = PointXYZ) {
    this.native = new __PCLCore__[`IterativeClosestPoint${_PT.name}`]();
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

  public hasConverged(): boolean {
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

  public align(output?: PointCloud<T>) {
    return wrapPointCloud<T>(this.native.align(output?.native ?? null));
  }
}

export default IterativeClosestPoint;
