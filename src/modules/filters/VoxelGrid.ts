import FilterBase from './FilterBase';
import { PointTypesUnion, PointTypes, PointTypesMerge } from '../point-types';

class VoxelGrid<
  T extends Partial<PointTypesUnion> = Partial<PointTypesMerge>,
> extends FilterBase<T> {
  constructor(pointType = PointTypes.PointXYZ) {
    super(new __PCLCore__[`VoxelGrid${pointType}`]());
  }

  public setLeafSize(lx: number, ly: number, lz: number) {
    return this.native.setLeafSize(lx, ly, lz);
  }
}

export default VoxelGrid;
