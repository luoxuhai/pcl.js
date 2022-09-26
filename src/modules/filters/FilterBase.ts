import {
  PointCloud,
  PointTypesUnion,
  PointTypesIntersection,
  Vector,
  TPointTypesUnion,
} from '../point-types';
import PCLBase from '../common/PCLBase';

abstract class FilterBase<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends PCLBase<T> {
  protected abstract PT?: TPointTypesUnion;

  public filter(cloud?: PointCloud<T>) {
    if (!this.PT) {
      return null;
    }

    const _cloud = cloud ?? new PointCloud<T>(this.PT);
    this.native.filter(_cloud.native);
    return _cloud;
  }

  public getRemovedIndices(): Vector<number> {
    return this.native.getRemovedIndices();
  }
}

export default FilterBase;
