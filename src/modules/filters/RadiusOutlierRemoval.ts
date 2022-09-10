import FilterIndices from './FilterIndices';
import {
  PointTypesUnion,
  TPointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
} from '../point-types';

class RadiusOutlierRemoval<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterIndices<T> {
  constructor(PT: TPointTypesUnion = PointXYZ, extractRemovedIndices = false) {
    super(
      new __PCLCore__[`RadiusOutlierRemoval${PT.name}`](extractRemovedIndices),
    );
  }

  public setRadiusSearch(radius: number) {
    return this.native.setRadiusSearch(radius);
  }

  public getRadiusSearch(): number {
    return this.native.getRadiusSearch();
  }

  public setMinNeighborsInRadius(minPts = 1) {
    return this.native.setMinNeighborsInRadius(minPts);
  }

  public getMinNeighborsInRadius(): number {
    return this.native.getMinNeighborsInRadius();
  }
}

export default RadiusOutlierRemoval;
