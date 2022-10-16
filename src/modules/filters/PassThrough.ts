import FilterIndices from './FilterIndices';
import { PointCloud, PointXYZ, XYZPointTypes } from '@/modules/common';
import { setInputXYZCloud } from '@/utils';

class PassThrough<T extends PointXYZ> extends FilterIndices<T> {
  protected _PT = PointXYZ;

  constructor(extractRemovedIndices = false) {
    super(new __PCLCore__.PassThroughPointXYZ(extractRemovedIndices));
  }

  public setInputCloud(cloud: PointCloud<XYZPointTypes>) {
    setInputXYZCloud(cloud, this._native.setInputCloud);
  }

  public setFilterFieldName(fieldName: string) {
    return this._native.setFilterFieldName(fieldName);
  }

  public getFilterFieldName(): string | null {
    return this._native.getFilterFieldName() as string | null;
  }

  public setFilterLimits(min: number, max: number) {
    return this._native.setFilterLimits(min, max);
  }

  public getFilterLimits(): [number, number] {
    return this._native.getFilterLimits();
  }
}

export default PassThrough;
