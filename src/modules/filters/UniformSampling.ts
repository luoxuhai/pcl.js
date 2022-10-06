import FilterBase from './FilterBase';
import { PointXYZ, XYZPointTypes, XYZPointTypesTypeof } from '@/modules/common/point-types';
import { UnionToIntersection } from '@/types/utils';

class UniformSampling<
  T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>,
> extends FilterBase<T> {
  constructor(protected _PT: XYZPointTypesTypeof = PointXYZ, extractRemovedIndices = false) {
    super(new __PCLCore__[`UniformSampling${_PT.name}`](extractRemovedIndices));
  }

  public setRadiusSearch(radius: number) {
    this._native.setRadiusSearch(radius);
  }
}

export default UniformSampling;
