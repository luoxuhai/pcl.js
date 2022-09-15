import FilterBase from './FilterBase';
import {
  PointTypesUnion,
  TPointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
} from '../point-types';

class UniformSampling<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterBase<T> {
  constructor(PT: TPointTypesUnion = PointXYZ, extractRemovedIndices = false) {
    super(new __PCLCore__[`UniformSampling${PT.name}`](extractRemovedIndices));
  }

  public setRadiusSearch(radius: number) {
    return this.native.setRadiusSearch(radius);
  }
}

export default UniformSampling;
