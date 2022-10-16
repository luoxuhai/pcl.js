import FilterIndices from './FilterIndices';
import { PointCloud, PointXYZ, XYZPointTypes } from '@/modules/common';
import { setInputXYZCloud } from '@/utils';

class LocalMaximum<T extends PointXYZ> extends FilterIndices<T> {
  protected _PT = PointXYZ;

  constructor(extractRemovedIndices = false) {
    super(new __PCLCore__.LocalMaximumPointXYZ(extractRemovedIndices));
  }

  public setInputCloud(cloud: PointCloud<XYZPointTypes>) {
    setInputXYZCloud(cloud, this._native.setInputCloud);
  }

  public setRadius(radius: number) {
    return this._native.setRadius(radius);
  }

  public getRadius(): number {
    return this._native.getRadius();
  }
}

export default LocalMaximum;
