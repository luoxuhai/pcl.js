import {
  PointCloud,
  PointTypesUnion,
  PointTypesIntersection,
  TPointTypesUnion,
  Indices,
} from '../point-types';
import PCLBase from '../common/PCLBase';

abstract class FilterBase<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends PCLBase<T> {
  protected abstract _PT?: TPointTypesUnion;

  public filter(cloud?: PointCloud<T>) {
    if (!this._PT) {
      return null;
    }

    const _cloud = cloud ?? new PointCloud<T>(this._PT);
    this._native.filter(_cloud._native);
    return _cloud;
  }

  public getRemovedIndices() {
    return new Indices(this._native.getRemovedIndices());
  }
}

export default FilterBase;
