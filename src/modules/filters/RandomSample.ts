import FilterIndices from './FilterIndices';
import {
  PointTypesUnion,
  TPointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
} from '@/modules/common/point-types';

class RandomSample<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterIndices<T> {
  constructor(
    protected _PT: TPointTypesUnion = PointXYZ,
    extractRemovedIndices = false,
  ) {
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
