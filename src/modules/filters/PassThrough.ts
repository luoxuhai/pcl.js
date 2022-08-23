class PassThrough {
  public native: any;

  constructor(extractRemovedIndices = false) {
    this.native = new pcl.Module.PassThrough(extractRemovedIndices);
  }

  public setInputCloud(cloud: PointCloud) {
    return this.native.setInputCloud(cloud);
  }

  public getInputCloud(): PointCloud | null {
    return this.native.getInputCloud() as PointCloud | null;
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

  // public getFilterLimits() {
  //   this.native.getFilterLimits(min, max)
  // }

  public setNegative(negative: boolean) {
    return this.native.setNegative(negative);
  }

  public getNegative(): boolean {
    return this.native.getNegative();
  }

  public setKeepOrganized(keepOrganized: boolean) {
    return this.native.setNegative(keepOrganized);
  }

  public getKeepOrganized(): boolean {
    return this.native.getKeepOrganized();
  }

  public setUserFilterValue(value: number) {
    return this.native.setNegative(value);
  }

  public filter(output: PointCloud) {
    return this.native.filter(output);
  }
}

export default PassThrough;
