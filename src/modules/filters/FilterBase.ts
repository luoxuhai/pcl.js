class FilterBase {
  public native: any;

  constructor(native: any) {
    this.native = native;
  }

  public setInputCloud(cloud: PointCloud): null {
    return this.native.setInputCloud(cloud);
  }

  public getInputCloud(): PointCloud | null {
    return this.native.getInputCloud() as PointCloud | null;
  }

  public filter(output: PointCloud) {
    return this.native.filter(output);
  }
}

export default FilterBase;
