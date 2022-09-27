import Keypoint from './Keypoint';
import {
  PointTypesUnion,
  PointTypesIntersection,
  TPointTypesUnion,
  PointXYZ,
} from '../point-types';

class ISSKeypoint3D<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends Keypoint<T> {
  constructor(_PT: TPointTypesUnion = PointXYZ, salientRadius = 0.0001) {
    super(new __PCLCore__[`ISSKeypoint3D${_PT.name}`](salientRadius));
  }

  public setSalientRadius(salientRadius: number) {
    this.native.setSalientRadius(salientRadius);
  }

  public setNonMaxRadius(radius: number) {
    this.native.setNonMaxRadius(radius);
  }

  public setNormalRadius(radius: number) {
    this.native.setNormalRadius(radius);
  }

  public setBorderRadius(radius: number) {
    this.native.setBorderRadius(radius);
  }

  public setMinNeighbors(minNeighbors: number) {
    this.native.setMinNeighbors(minNeighbors);
  }

  public setThreshold21(gamma21: number) {
    this.native.setThreshold21(gamma21);
  }

  public setThreshold32(gamma32: number) {
    this.native.setThreshold32(gamma32);
  }

  public setAngleThreshold(angle: number) {
    this.native.setAngleThreshold(angle);
  }
}

export default ISSKeypoint3D;
