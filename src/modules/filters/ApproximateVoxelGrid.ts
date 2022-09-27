import FilterBase from './FilterBase';
import {
  PointTypesUnion,
  TPointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
} from '../point-types';

class ApproximateVoxelGrid<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterBase<T> {
  constructor(protected _PT: TPointTypesUnion = PointXYZ) {
    super(new __PCLCore__[`ApproximateVoxelGrid${_PT.name}`]());
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
}

export default ApproximateVoxelGrid;
