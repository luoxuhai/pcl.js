import FilterBase from './FilterBase';
import { PointXYZ, XYZPointTypes, PointCloud } from '@/modules/common';
import { setInputXYZCloud } from '@/utils';

class ApproximateVoxelGrid<T extends PointXYZ> extends FilterBase<T> {
  protected _PT = PointXYZ;

  constructor() {
    super(new __PCLCore__.ApproximateVoxelGridPointXYZ());
  }

  public setInputCloud(cloud: PointCloud<XYZPointTypes>): void {
    setInputXYZCloud(cloud, this._native.setInputCloud);
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
