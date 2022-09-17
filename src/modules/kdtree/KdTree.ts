import {
  PointCloud,
  wrapPointCloud,
  PointTypesUnion,
  PointTypesIntersection,
} from '../point-types';

class KdTree<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> {
  public native: Emscripten.NativeAPI;

  constructor(native: Emscripten.NativeAPI) {
    this.native = native;
  }

  public setInputCloud(cloud: PointCloud<T>) {
    this.native.setInputCloud(cloud.native);
  }

  public getInputCloud() {
    return wrapPointCloud<T>(this.native.getInputCloud());
  }
}

export default KdTree;
