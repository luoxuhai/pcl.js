import FilterBase from './FilterBase';
import { XYZPointTypes, XYZPointTypesTypeof, PointXYZ } from '@/modules/common/point-types';
import { UnionToIntersection } from '@/types/utils';

class ApproximateVoxelGrid<
  T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>,
> extends FilterBase<T> {
  constructor(protected _PT: XYZPointTypesTypeof = PointXYZ) {
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
