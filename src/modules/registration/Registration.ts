import PCLBase from '@/modules/common/PCLBase';
import { PointCloud, wrapPointCloud } from '@/modules/common/PointCloud';
import { XYZPointTypes, PointXYZ } from '@/modules/common/point-types';
import { toXYZPointCloud } from '@/modules/common/utils';
import { Emscripten } from '@/types';

class Registration<T extends XYZPointTypes> extends PCLBase<T> {
  constructor(native: Emscripten.NativeAPI) {
    super(native);
  }

  /**
   * Only supports `PointXYZ` type,
   * if it is not `PointXYZ` type, it will use `toXYZPointCloud` method to convert to `PointXYZ`
   */
  public setInputSource(cloud: PointCloud<T>) {
    if (cloud._PT === PointXYZ) {
      this._native.setInputSource(cloud._native);
    } else {
      const cloudXYZ = new PointCloud<PointXYZ>();
      toXYZPointCloud(cloud, cloudXYZ);
      this._native.setInputSource(cloudXYZ._native);
      cloudXYZ.manager.delete();
    }
  }

  public getInputSource() {
    return wrapPointCloud<PointXYZ>(this._native.getInputSource());
  }

  /**
   * Only supports `PointXYZ` type,
   * if it is not `PointXYZ` type, it will use `toXYZPointCloud` method to convert to `PointXYZ`
   */
  public setInputTarget(cloud: PointCloud<T>) {
    if (cloud._PT === PointXYZ) {
      this._native.setInputTarget(cloud._native);
    } else {
      const cloudXYZ = new PointCloud<PointXYZ>();
      toXYZPointCloud(cloud, cloudXYZ);
      this._native.setInputTarget(cloudXYZ._native);
      cloudXYZ.manager.delete();
    }
  }

  public getInputTarget() {
    return wrapPointCloud<PointXYZ>(this._native.getInputTarget());
  }

  public getFitnessScore(maxRange = Number.MAX_VALUE): number {
    return this._native.getFitnessScore(maxRange);
  }

  public hasConverged(): boolean {
    return this._native.hasConverged();
  }

  public setMaximumIterations(maxIterations: number) {
    this._native.setMaximumIterations(maxIterations);
  }

  public getMaximumIterations(): number {
    return this._native.getMaximumIterations();
  }

  public setRANSACIterations(ransacIterations: number) {
    this._native.setRANSACIterations(ransacIterations);
  }

  public getRANSACIterations(): number {
    return this._native.getRANSACIterations();
  }

  public setRANSACOutlierRejectionThreshold(inlierThreshold: number) {
    this._native.setRANSACOutlierRejectionThreshold(inlierThreshold);
  }

  public getRANSACOutlierRejectionThreshold(): number {
    return this._native.getRANSACOutlierRejectionThreshold();
  }

  public setMaxCorrespondenceDistance(distanceThreshold: number) {
    this._native.setMaxCorrespondenceDistance(distanceThreshold);
  }

  public getMaxCorrespondenceDistance(): number {
    return this._native.getMaxCorrespondenceDistance();
  }

  public setTransformationEpsilon(epsilon: number) {
    this._native.setTransformationEpsilon(epsilon);
  }

  public getTransformationEpsilon(): number {
    return this._native.getTransformationEpsilon();
  }

  public setTransformationRotationEpsilon(epsilon: number) {
    this._native.setTransformationRotationEpsilon(epsilon);
  }

  public getTransformationRotationEpsilon(): number {
    return this._native.getTransformationRotationEpsilon();
  }

  public setEuclideanFitnessEpsilon(epsilon: number) {
    this._native.setEuclideanFitnessEpsilon(epsilon);
  }

  public getEuclideanFitnessEpsilon(): number {
    return this._native.getEuclideanFitnessEpsilon();
  }

  public initCompute(): boolean {
    return this._native.initCompute();
  }

  public initComputeReciprocal(): boolean {
    return this._native.initComputeReciprocal();
  }

  public align(output: PointCloud<T>,matrix: [] = []) {
    this._native.align(output._native,matrix);
  }

  public getFinalTransformation(){
    return this._native.getFinalTransformation();
  }
}

export default Registration;
