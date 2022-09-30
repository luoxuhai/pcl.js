import FilterBase from './FilterBase';
import {
  PointTypesUnion,
  TPointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
} from '@/modules/common/point-types';

class UniformSampling<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterBase<T> {
  constructor(
    protected _PT: TPointTypesUnion = PointXYZ,
    extractRemovedIndices = false,
  ) {
    super(new __PCLCore__[`UniformSampling${_PT.name}`](extractRemovedIndices));
  }

  public setRadiusSearch(radius: number) {
    return this._native.setRadiusSearch(radius);
  }
}

export default UniformSampling;
