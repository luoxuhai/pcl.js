import FeatureFromNormals from './FeatureFromNormals';
import { XYZPointTypes, Normal, FPFHSignature33 } from '@/modules/common/point-types';

class FPFHEstimation<
  PointInT extends XYZPointTypes,
  PointNT extends Normal = Normal,
  PointOutT extends FPFHSignature33 = FPFHSignature33,
> extends FeatureFromNormals<PointInT, PointNT, PointOutT> {
  private _nrBinsF1 = 11;
  private _nrBinsF2 = 11;
  private _nrBinsF3 = 11;

  constructor() {
    const native = new __PCLCore__.FPFHEstimationPointXYZNormalFPFHSignature33();
    super(native);
  }

  public setNrSubdivisions(nrBinsF1: number, nrBinsF2: number, nrBinsF3: number) {
    this._native.setViewPoint(nrBinsF1, nrBinsF2, nrBinsF3);
    this._nrBinsF1 = nrBinsF1;
    this._nrBinsF2 = nrBinsF2;
    this._nrBinsF3 = nrBinsF3;
  }

  public getNrSubdivisions() {
    return {
      nrBinsF1: this._nrBinsF1,
      nrBinsF2: this._nrBinsF2,
      nrBinsF3: this._nrBinsF3,
    };
  }
}

export default FPFHEstimation;
