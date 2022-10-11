import { wrapPointCloud } from '@/modules/common/PointCloud';
import { XYZPointTypes } from '@/modules/common/point-types';
import { Emscripten } from '@/types';
import Manager from '@/modules/common/Manager';

class Search<T extends XYZPointTypes> {
  public manager = new Manager(this._native);

  constructor(public _native: Emscripten.NativeAPI) {}

  public getInputCloud() {
    return wrapPointCloud<T>(this._native.getInputCloud());
  }

  public getName() {
    this._native.getName();
  }

  public getSortedResults() {
    this._native.getSortedResults();
  }
}

export default Search;
