import {
  wrapPointCloud,
  PointTypesUnion,
  PointTypesIntersection,
} from '../point-types';

class Search<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> {
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
