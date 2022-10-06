import { PointCloud, wrapPointCloud } from '@/modules/common/PointCloud';
import { XYZPointTypes } from '@/modules/common/point-types';
import { Emscripten } from '@/types';

class KdTree<T extends XYZPointTypes> {
  constructor(public _native: Emscripten.NativeAPI) {}

  public setInputCloud(cloud: PointCloud<T>) {
    this._native.setInputCloud(cloud._native);
  }

  public getInputCloud() {
    return wrapPointCloud<T>(this._native.getInputCloud());
  }
}

export default KdTree;
