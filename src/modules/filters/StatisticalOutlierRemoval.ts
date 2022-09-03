import FilterIndices from './FilterIndices';
import { PointTypesUnion, PointTypes, PointTypesMerge } from '../point-types';

class StatisticalOutlierRemoval<
  T extends Partial<PointTypesUnion> = Partial<PointTypesMerge>,
> extends FilterIndices<T> {
  constructor(pointType = PointTypes.PointXYZ, extractRemovedIndices = false) {
    super(
      new __PCLCore__[`StatisticalOutlierRemoval${pointType}`](
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
