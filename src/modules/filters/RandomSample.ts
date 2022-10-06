import FilterIndices from './FilterIndices';
import { XYZPointTypes, XYZPointTypesTypeof, PointXYZ } from '@/modules/common/point-types';
import { UnionToIntersection } from '@/types/utils';

class RandomSample<
  T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>,
> extends FilterIndices<T> {
  constructor(protected _PT: XYZPointTypesTypeof = PointXYZ, extractRemovedIndices = false) {
    super(new __PCLCore__[`RandomSample${_PT.name}`](extractRemovedIndices));
  }

  public setSample(sample: number) {
    return this._native.setSample(sample);
  }

  public getSample(): number {
    return this._native.getSample();
  }

  public setSeed(seed: number) {
    return this._native.setSeed(seed);
  }

  public getSeed(): number {
    return this._native.getSeed();
  }
}

export default RandomSample;
