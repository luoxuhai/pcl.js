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
  constructor(
    protected _PT: TPointTypesUnion = PointXYZ,
    extractRemovedIndices = false,
  ) {
    super(
      new __PCLCore__[`RadiusOutlierRemoval${_PT.name}`](extractRemovedIndices),
    );
  }

  public setRadiusSearch(radius: number) {
    return this._native.setRadiusSearch(radius);
  }

  public getRadiusSearch(): number {
    return this._native.getRadiusSearch();
  }

  public setMinNeighborsInRadius(minPts = 1) {
    return this._native.setMinNeighborsInRadius(minPts);
  }

  public getMinNeighborsInRadius(): number {
    return this._native.getMinNeighborsInRadius();
  }
}

export default RadiusOutlierRemoval;
