import FilterIndices from './FilterIndices';
import { PointXYZ, XYZPointTypes, PointCloud } from '@/modules/common';
import { setInputXYZCloud } from '@/utils';

class GridMinimum<T extends PointXYZ> extends FilterIndices<T> {
  protected _PT = PointXYZ;

  constructor(resolution = 0) {
    super(new __PCLCore__.GridMinimumPointXYZ(resolution));
  }

  public setInputCloud(cloud: PointCloud<XYZPointTypes>) {
    setInputXYZCloud(cloud, this._native.setInputCloud);
  }

  public setResolution(resolution: number) {
    return this._native.setResolution(resolution);
  }

  public getResolution(): number {
    return this._native.getResolution();
  }
}

export default GridMinimum;
