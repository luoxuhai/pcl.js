import FilterBase from './FilterBase';
import {
  PointTypesUnion,
  TPointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
} from '../point-types';

class VoxelGrid<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterBase<T> {
  constructor(PT: TPointTypesUnion = PointXYZ) {
    super(new __PCLCore__[`VoxelGrid${PT.name}`]());
  }

  public setLeafSize(lx: number, ly: number, lz: number) {
    return this.native.setLeafSize(lx, ly, lz);
  }
}

export default VoxelGrid;
