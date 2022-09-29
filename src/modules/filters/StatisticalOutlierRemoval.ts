import FilterIndices from './FilterIndices';
import {
  PointTypesUnion,
  PointXYZ,
  PointTypesIntersection,
  TPointTypesUnion,
} from '../point-types';

class StatisticalOutlierRemoval<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends FilterIndices<T> {
  constructor(
    protected _PT: TPointTypesUnion = PointXYZ,
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
