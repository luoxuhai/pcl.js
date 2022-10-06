import FilterIndices from './FilterIndices';
import { XYZPointTypes, XYZPointTypesTypeof, PointXYZ } from '@/modules/common/point-types';

class LocalMaximum<
  T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>,
> extends FilterIndices<T> {
  constructor(protected _PT: XYZPointTypesTypeof = PointXYZ, extractRemovedIndices = false) {
    super(new __PCLCore__[`LocalMaximum${_PT.name}`](extractRemovedIndices));
  }

  public setRadius(radius: number) {
    return this._native.setRadius(radius);
  }

  public getRadius(): number {
    return this._native.getRadius();
  }
}

export default LocalMaximum;
