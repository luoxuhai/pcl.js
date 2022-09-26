import {
  PointCloud,
  wrapPointCloud,
  PointTypesUnion,
  PointTypesIntersection,
} from '../point-types';

class KdTree<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> {
  constructor(public native: Emscripten.NativeAPI) {}

  public setInputCloud(cloud: PointCloud<T>) {
    this.native.setInputCloud(cloud.native);
  }

  public getInputCloud() {
    return wrapPointCloud<T>(this.native.getInputCloud());
  }
}

export default KdTree;
