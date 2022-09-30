import { PointCloud, wrapPointCloud } from '@/modules/common/PointCloud';
import {
  PointTypesUnion,
  PointTypesIntersection,
} from '@/modules/common/point-types';

class KdTree<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> {
  constructor(public _native: Emscripten.NativeAPI) {}

  public setInputCloud(cloud: PointCloud<T>) {
    this._native.setInputCloud(cloud._native);
  }

  public getInputCloud() {
    return wrapPointCloud<T>(this._native.getInputCloud());
  }
}

export default KdTree;
