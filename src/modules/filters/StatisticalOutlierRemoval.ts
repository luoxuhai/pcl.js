import FilterIndices from './FilterIndices';
import {
  PointXYZ,
  XYZPointTypes,
  XYZPointTypesTypeof,
} from '@/modules/common/point-types';

class StatisticalOutlierRemoval<
  T extends XYZPointTypes = PointXYZ &
    Partial<UnionToIntersection<XYZPointTypes>>,
> extends FilterIndices<T> {
  constructor(
    protected _PT: XYZPointTypesTypeof = PointXYZ,
    extractRemovedIndices = false,
  ) {
    super(
      new __PCLCore__[`StatisticalOutlierRemoval${_PT.name}`](
        extractRemovedIndices,
      ),
    );
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
