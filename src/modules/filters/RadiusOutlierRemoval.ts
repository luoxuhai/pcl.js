import FilterIndices from './FilterIndices';
import { PointTypesUnion, PointTypes, PointTypesMerge } from '../point-types';

class RadiusOutlierRemoval<
  T extends Partial<PointTypesUnion> = Partial<PointTypesMerge>,
> extends FilterIndices<T> {
  constructor(pointType = PointTypes.PointXYZ, extractRemovedIndices = false) {
    super(
      new __PCLCore__[`RadiusOutlierRemoval${pointType}`](
        extractRemovedIndices,
      ),
    );
  }

  public setRadiusSearch(radius: number) {
    return this.native.setRadiusSearch(radius);
  }

  public getRadiusSearch(): number {
    return this.native.getRadiusSearch();
  }

  public setMinNeighborsInRadius(minPts: number) {
    return this.native.setMinNeighborsInRadius(minPts);
  }

  public getMinNeighborsInRadius(): number {
    return this.native.getMinNeighborsInRadius();
  }
}

export default RadiusOutlierRemoval;
