import FilterIndices from './FilterIndices';
import {
  PointTypesUnion,
  TPointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
} from '../point-types';

class GridMinimum<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterIndices<T> {
  constructor(protected PT: TPointTypesUnion = PointXYZ, resolution = 0) {
    super(new __PCLCore__[`GridMinimum${PT.name}`](resolution));
  }

  public setResolution(resolution: number) {
    return this.native.setResolution(resolution);
  }

  public getResolution(): number {
    return this.native.getResolution();
  }
}

export default GridMinimum;
