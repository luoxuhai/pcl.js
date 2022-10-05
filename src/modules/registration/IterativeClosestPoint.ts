import { PointCloud, wrapPointCloud } from '@/modules/common/PointCloud';
import {
  XYZPointTypes,
  XYZPointTypesTypeof,
  PointXYZ,
} from '@/modules/common/point-types';

class IterativeClosestPoint<
  T extends XYZPointTypes = PointXYZ &
    Partial<UnionToIntersection<XYZPointTypes>>,
> {
  public _native: Emscripten.NativeAPI;

  constructor(_PT: XYZPointTypesTypeof = PointXYZ) {
    this._native = new __PCLCore__[`IterativeClosestPoint${_PT.name}`]();
  }

  public setInputSource(cloud: PointCloud<T>) {
    this._native.setInputSource(cloud._native);
  }

  public setInputTarget(cloud: PointCloud<T>) {
    this._native.setInputTarget(cloud._native);
  }

  public getFinalTransformation(): string | null {
    return this._native.getFinalTransformation();
  }

  public getFitnessScore(): number {
    return this._native.getFitnessScore();
  }

  public hasConverged(): boolean {
    return this._native.hasConverged();
  }

  public setUseReciprocalCorrespondences(useReciprocalCorrespondence: boolean) {
    this._native.setUseReciprocalCorrespondences(useReciprocalCorrespondence);
  }

  public getUseReciprocalCorrespondences(): boolean {
    return this._native.getUseReciprocalCorrespondences();
  }

  public align(output?: PointCloud<T>) {
    return wrapPointCloud<T>(this._native.align(output?._native ?? null));
  }
}

export default IterativeClosestPoint;
