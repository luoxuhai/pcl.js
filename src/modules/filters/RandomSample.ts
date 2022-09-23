import FilterIndices from './FilterIndices';
import {
  PointTypesUnion,
  TPointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
} from '../point-types';

class RandomSample<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterIndices<T> {
  constructor(
    protected PT: TPointTypesUnion = PointXYZ,
    extractRemovedIndices = false,
  ) {
    super(new __PCLCore__[`RandomSample${PT.name}`](extractRemovedIndices));
  }

  public setSample(sample: number) {
    return this.native.setSample(sample);
  }

  public getSample(): number {
    return this.native.getSample();
  }

  public setSeed(seed: number) {
    return this.native.setSeed(seed);
  }

  public getSeed(): number {
    return this.native.getSeed();
  }
}

export default RandomSample;
