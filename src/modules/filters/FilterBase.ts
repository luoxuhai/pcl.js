import { PointCloud, createPointCloud } from '../point-types';

class FilterBase<T> {
  public native: Emscripten.NativeAPI;

  constructor(native: Emscripten.NativeAPI) {
    this.native = native;
  }

  public setInputCloud(cloud: PointCloud<T>): null {
    return this.native.setInputCloud(cloud.native);
  }

  public getInputCloud() {
    return createPointCloud<T>(this.native.getInputCloud());
  }

  public filter(cloud?: PointCloud<T>) {
    return createPointCloud<T>(this.native.filter(cloud?.native ?? null));
  }
}

export default FilterBase;
