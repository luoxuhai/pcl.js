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
    protected PT: TPointTypesUnion = PointXYZ,
    extractRemovedIndices = false,
  ) {
    super(
      new __PCLCore__[`StatisticalOutlierRemoval${PT.name}`](
        extractRemovedIndices,
      ),
    );
  }

  public setMeanK(nrK: number) {
    return this.native.setMeanK(nrK);
  }

  public getMeanK(): number {
    return this.native.getMeanK();
  }

  public setStddevMulThresh(stddevMult: number) {
    return this.native.setStddevMulThresh(stddevMult);
  }

  public getStddevMulThresh(): number {
    return this.native.getStddevMulThresh();
  }
}

export default StatisticalOutlierRemoval;
