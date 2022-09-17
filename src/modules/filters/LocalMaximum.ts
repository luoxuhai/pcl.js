import FilterIndices from './FilterIndices';
import {
  PointTypesUnion,
  TPointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
} from '../point-types';

class LocalMaximum<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterIndices<T> {
  constructor(PT: TPointTypesUnion = PointXYZ, extractRemovedIndices = false) {
    super(new __PCLCore__[`LocalMaximum${PT.name}`](extractRemovedIndices));
  }

  public setRadius(radius: number) {
    return this.native.setRadius(radius);
  }

  public getRadius(): number {
    return this.native.getRadius();
  }
}

export default LocalMaximum;
