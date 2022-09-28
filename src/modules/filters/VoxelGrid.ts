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
  constructor(protected _PT: TPointTypesUnion = PointXYZ) {
    super(new __PCLCore__[`VoxelGrid${_PT.name}`]());
  }

  public setLeafSize(lx: number, ly: number, lz: number) {
    return this._native.setLeafSize(lx, ly, lz);
  }

  public setDownsampleAllData(downsample: boolean) {
    return this._native.setDownsampleAllData(downsample);
  }

  public getDownsampleAllData(): boolean {
    return this._native.getDownsampleAllData();
  }

  public setMinimumPointsNumberPerVoxel(minPointsPerVoxel: number) {
    return this._native.setMinimumPointsNumberPerVoxel(minPointsPerVoxel);
  }

  public getMinimumPointsNumberPerVoxel(): number {
    return this._native.getMinimumPointsNumberPerVoxel();
  }
}

export default VoxelGrid;
