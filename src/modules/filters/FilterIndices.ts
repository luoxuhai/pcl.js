import FilterBase from './FilterBase';
import {
  PointTypesUnion,
  PointTypesIntersection,
  TPointTypesUnion,
} from '../point-types';

class FilterIndices<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterBase<T> {
  protected _PT?: TPointTypesUnion;

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
