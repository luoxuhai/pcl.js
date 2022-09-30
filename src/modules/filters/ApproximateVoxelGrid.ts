import FilterBase from './FilterBase';
import {
  PointTypesUnion,
  TPointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
} from '@/modules/common/point-types';

class ApproximateVoxelGrid<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterBase<T> {
  constructor(protected _PT: TPointTypesUnion = PointXYZ) {
    super(new __PCLCore__[`ApproximateVoxelGrid${_PT.name}`]());
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
}

export default ApproximateVoxelGrid;
