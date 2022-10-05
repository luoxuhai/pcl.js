import Keypoint from './Keypoint';
import {
  XYZPointTypes,
  XYZPointTypesTypeof,
  PointXYZ,
} from '@/modules/common/point-types';

class ISSKeypoint3D<
  T extends XYZPointTypes = PointXYZ &
    Partial<UnionToIntersection<XYZPointTypes>>,
> extends Keypoint<T> {
  constructor(_PT: XYZPointTypesTypeof = PointXYZ, salientRadius = 0.0001) {
    super(new __PCLCore__[`ISSKeypoint3D${_PT.name}`](salientRadius));
  }

  public setSalientRadius(salientRadius: number) {
    this._native.setSalientRadius(salientRadius);
  }

  public setNonMaxRadius(radius: number) {
    this._native.setNonMaxRadius(radius);
  }

  public setNormalRadius(radius: number) {
    this._native.setNormalRadius(radius);
  }

  public setBorderRadius(radius: number) {
    this._native.setBorderRadius(radius);
  }

  public setMinNeighbors(minNeighbors: number) {
    this._native.setMinNeighbors(minNeighbors);
  }

  public setThreshold21(gamma21: number) {
    this._native.setThreshold21(gamma21);
  }

  public setThreshold32(gamma32: number) {
    this._native.setThreshold32(gamma32);
  }

  public setAngleThreshold(angle: number) {
    this._native.setAngleThreshold(angle);
  }
}

export default ISSKeypoint3D;
