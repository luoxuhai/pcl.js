import { PointCloud, wrapPointCloud } from './PointCloud';
import { PointTypes } from './point-types';
import { Emscripten } from '@/types';
import Manager from './Manager';

class PCLBase<T extends PointTypes> {
  public manager = new Manager(this._native);

  constructor(public _native: Emscripten.NativeAPI) {}

  public setInputCloud(cloud: PointCloud<T>): null {
    return this._native.setInputCloud(cloud._native);
  }

  public getInputCloud() {
    return wrapPointCloud<T>(this._native.getInputCloud());
  }
}

export default PCLBase;
