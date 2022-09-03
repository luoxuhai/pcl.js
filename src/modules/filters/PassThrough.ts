import FilterIndices from './FilterIndices';
import { PointTypesUnion, PointTypes, PointTypesMerge } from '../point-types';

class PassThrough<
  T extends Partial<PointTypesUnion> = Partial<PointTypesMerge>,
> extends FilterIndices<T> {
  constructor(pointType = PointTypes.PointXYZ, extractRemovedIndices = false) {
    super(new __PCLCore__[`PassThrough${pointType}`](extractRemovedIndices));
  }

  public setFilterFieldName(fieldName: string) {
    return this.native.getInputCloud(fieldName);
  }

  public getFilterFieldName(): string | null {
    return this.native.getFilterFieldName() as string | null;
  }

  public setFilterLimits(min: number, max: number) {
    return this.native.setFilterLimits(min, max);
  }

  public getFilterLimits(): [number, number] {
    return this.native.getFilterLimits();
  }
}

export default PassThrough;
