import FilterBase from './FilterBase';
import { PointCloud, PointXYZ, XYZPointTypes } from '@/modules/common';
import { setInputXYZCloud } from '@/utils';

class UniformSampling<T extends PointXYZ> extends FilterBase<T> {
  protected _PT = PointXYZ;

  constructor(extractRemovedIndices = false) {
    super(new __PCLCore__.UniformSamplingPointXYZ(extractRemovedIndices));
  }

  public setInputCloud(cloud: PointCloud<XYZPointTypes>) {
    setInputXYZCloud(cloud, this._native.setInputCloud);
  }

  public setRadiusSearch(radius: number) {
    this._native.setRadiusSearch(radius);
  }
}

export default UniformSampling;
