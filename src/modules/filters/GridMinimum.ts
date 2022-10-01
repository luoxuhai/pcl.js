import FilterIndices from './FilterIndices';
import {
  PointTypesUnion,
  TPointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
} from '@/modules/common/point-types';

class GridMinimum<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterIndices<T> {
  constructor(protected _PT: TPointTypesUnion = PointXYZ, resolution = 0) {
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
