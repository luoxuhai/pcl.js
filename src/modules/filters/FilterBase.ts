import { PointCloud, Indices } from '@/modules/common/PointCloud';
import { PointTypes, PointTypesTypeof } from '@/modules/common/point-types';
import PCLBase from '@/modules/common/PCLBase';

abstract class FilterBase<T extends PointTypes> extends PCLBase<T> {
  protected abstract _PT?: PointTypesTypeof;

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
