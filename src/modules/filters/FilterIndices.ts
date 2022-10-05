import FilterBase from './FilterBase';
import { PointTypes, PointTypesTypeof } from '@/modules/common/point-types';

class FilterIndices<T extends PointTypes> extends FilterBase<T> {
  protected _PT?: PointTypesTypeof;

  public setNegative(negative = false) {
    this._native.setNegative(negative);
  }

  public getNegative(): boolean {
    return this._native.getNegative();
  }

  public setKeepOrganized(keepOrganized = false) {
    this._native.setNegative(keepOrganized);
  }

  public getKeepOrganized(): boolean {
    return this._native.getKeepOrganized();
  }

  public setUserFilterValue(value: number) {
    this._native.setNegative(value);
  }
}

export default FilterIndices;
