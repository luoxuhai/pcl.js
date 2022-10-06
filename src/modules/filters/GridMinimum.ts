import FilterIndices from './FilterIndices';
import { XYZPointTypes, XYZPointTypesTypeof, PointXYZ } from '@/modules/common/point-types';
import { UnionToIntersection } from '@/types/utils';

class GridMinimum<
  T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>,
> extends FilterIndices<T> {
  constructor(protected _PT: XYZPointTypesTypeof = PointXYZ, resolution = 0) {
    super(new __PCLCore__[`GridMinimum${_PT.name}`](resolution));
  }

  public setResolution(resolution: number) {
    return this._native.setResolution(resolution);
  }

  public getResolution(): number {
    return this._native.getResolution();
  }
}

export default GridMinimum;
