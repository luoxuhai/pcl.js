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

  public setDownsampleAllData(downsample: boolean) {
    return this.native.setDownsampleAllData(downsample);
  }

  public getDownsampleAllData(): boolean {
    return this.native.getDownsampleAllData();
  }

  public setMinimumPointsNumberPerVoxel(minPointsPerVoxel: number) {
    return this.native.setMinimumPointsNumberPerVoxel(minPointsPerVoxel);
  }

  public getMinimumPointsNumberPerVoxel(): number {
    return this.native.getMinimumPointsNumberPerVoxel();
  }
}

export default VoxelGrid;
