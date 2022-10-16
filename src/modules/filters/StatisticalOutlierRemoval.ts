import FilterIndices from './FilterIndices';
import { PointCloud, PointXYZ, XYZPointTypes } from '@/modules/common';
import { setInputXYZCloud } from '@/utils';

class StatisticalOutlierRemoval<T extends PointXYZ> extends FilterIndices<T> {
  protected _PT = PointXYZ;

  constructor(extractRemovedIndices = false) {
    super(new __PCLCore__.StatisticalOutlierRemovalPointXYZ(extractRemovedIndices));
  }

  public setInputCloud(cloud: PointCloud<XYZPointTypes>) {
    setInputXYZCloud(cloud, this._native.setInputCloud);
  }

  public setMeanK(nrK: number) {
    return this._native.setMeanK(nrK);
  }

  public getMeanK(): number {
    return this._native.getMeanK();
  }

  public setStddevMulThresh(stddevMult: number) {
    return this._native.setStddevMulThresh(stddevMult);
  }

  public getStddevMulThresh(): number {
    return this._native.getStddevMulThresh();
  }
}

export default StatisticalOutlierRemoval;
