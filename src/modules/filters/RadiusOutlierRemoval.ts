import FilterIndices from './FilterIndices';
import { PointCloud, PointXYZ, XYZPointTypes } from '@/modules/common';
import { setInputXYZCloud } from '@/utils';

class RadiusOutlierRemoval<T extends PointXYZ> extends FilterIndices<T> {
  protected _PT = PointXYZ;

  constructor(extractRemovedIndices = false) {
    super(new __PCLCore__.RadiusOutlierRemovalPointXYZ(extractRemovedIndices));
  }

  public setInputCloud(cloud: PointCloud<XYZPointTypes>) {
    setInputXYZCloud(cloud, this._native.setInputCloud);
  }

  public setRadiusSearch(radius: number) {
    return this._native.setRadiusSearch(radius);
  }

  public getRadiusSearch(): number {
    return this._native.getRadiusSearch();
  }

  public setMinNeighborsInRadius(minPts = 1) {
    return this._native.setMinNeighborsInRadius(minPts);
  }

  public getMinNeighborsInRadius(): number {
    return this._native.getMinNeighborsInRadius();
  }
}

export default RadiusOutlierRemoval;
