import FilterIndices from './FilterIndices';

class StatisticalOutlierRemoval extends FilterIndices {
  constructor(extractRemovedIndices = false) {
    super(new __PCLCore__.StatisticalOutlierRemoval(extractRemovedIndices));
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
