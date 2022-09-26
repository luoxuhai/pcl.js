import {
  wrapPointCloud,
  PointTypesUnion,
  PointTypesIntersection,
} from '../point-types';

class Search<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> {
  constructor(public native: Emscripten.NativeAPI) {}

  public getInputCloud() {
    return wrapPointCloud<T>(this.native.getInputCloud());
  }

  public getName() {
    this.native.getName();
  }

  public getSortedResults() {
    this.native.getSortedResults();
  }
}

export default Search;
