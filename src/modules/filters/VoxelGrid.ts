import FilterBase from './FilterBase';
import { PointCloud, PointXYZ, XYZPointTypes } from '@/modules/common';
import { setInputXYZCloud } from '@/utils';

class VoxelGrid<T extends PointXYZ> extends FilterBase<T> {
  protected _PT = PointXYZ;

  constructor() {
    super(new __PCLCore__.VoxelGridPointXYZ());
  }

  public setInputCloud(cloud: PointCloud<XYZPointTypes>) {
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

  public setMinimumPointsNumberPerVoxel(minPointsPerVoxel: number) {
    return this._native.setMinimumPointsNumberPerVoxel(minPointsPerVoxel);
  }

  public getMinimumPointsNumberPerVoxel(): number {
    return this._native.getMinimumPointsNumberPerVoxel();
  }
}

export default VoxelGrid;
