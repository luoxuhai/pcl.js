import PCLBase from '@/modules/common/PCLBase';
import { PointCloud } from '@/modules/common/PointCloud';
import { XYZPointTypes, FPFHSignature33, Normal } from '@/modules/common/point-types';
import Search from '@/modules/search/Search';
import { Emscripten } from '@/types';

class Feature<
  PointInT extends XYZPointTypes,
  PointOutT extends FPFHSignature33 | Normal = Normal,
> extends PCLBase<PointInT> {
  private _searchMethod?: Search<PointInT>;

  constructor(native: Emscripten.NativeAPI) {
    super(native);
  }

  public setSearchSurface(cloud: PointCloud<PointInT>) {
    this._native.setSearchSurface(cloud._native);
  }

  public setSearchMethod(searchMethod: Search<PointInT>) {
    this._native.setSearchMethod(searchMethod._native);
    this._searchMethod = searchMethod;
  }

  public getSearchMethod() {
    return this._searchMethod;
  }

  public getSearchParameter(): number {
    return this._native.getSearchParameter();
  }

  public setKSearch(k: number) {
    this._native.setKSearch(k);
  }

  public getKSearch(): number {
    return this._native.getKSearch();
  }

  public setRadiusSearch(radius: number) {
    this._native.setRadiusSearch(radius);
  }

  public getRadiusSearch() {
    return this._native.getRadiusSearch();
  }

  public compute(cloud: PointCloud<PointOutT>) {
    this._native.compute(cloud._native);
  }
}

export default Feature;
