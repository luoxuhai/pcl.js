import FilterBase from './FilterBase';
import { PointTypesUnion, PointTypesIntersection } from '../point-types';

class FilterIndices<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterBase<T> {
  public setNegative(negative: boolean) {
    this.native.setNegative(negative);
  }

  public getNegative(): boolean {
    return this.native.getNegative();
  }

  public setKeepOrganized(keepOrganized: boolean) {
    this.native.setNegative(keepOrganized);
  }

  public getKeepOrganized(): boolean {
    return this.native.getKeepOrganized();
  }

  public setUserFilterValue(value: number) {
    this.native.setNegative(value);
  }
}

export default FilterIndices;
