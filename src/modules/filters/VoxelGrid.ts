import FilterBase from './FilterBase';
import {
  XYZPointTypes,
  XYZPointTypesTypeof,
  PointXYZ,
} from '@/modules/common/point-types';

class VoxelGrid<
  T extends XYZPointTypes = PointXYZ &
    Partial<UnionToIntersection<XYZPointTypes>>,
> extends FilterBase<T> {
  constructor(protected _PT: XYZPointTypesTypeof = PointXYZ) {
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
