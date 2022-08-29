import FilterIndices from './FilterIndices';

class PassThrough extends FilterIndices {
  constructor(extractRemovedIndices = false) {
    super(new __PCLCore__.PassThrough(extractRemovedIndices));
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
