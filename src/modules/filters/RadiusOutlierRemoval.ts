import FilterIndices from './FilterIndices';
import { XYZPointTypes, XYZPointTypesTypeof, PointXYZ } from '@/modules/common/point-types';

class RadiusOutlierRemoval<
  T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>,
> extends FilterIndices<T> {
  constructor(protected _PT: XYZPointTypesTypeof = PointXYZ, extractRemovedIndices = false) {
    super(new __PCLCore__[`RadiusOutlierRemoval${_PT.name}`](extractRemovedIndices));
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
