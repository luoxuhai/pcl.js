import Feature from './Feature';
import { XYZPointTypes, Normal } from '@/modules/common/point-types';

class NormalEstimation<
  PointInT extends XYZPointTypes,
  PointOutT extends Normal = Normal,
> extends Feature<PointInT, PointOutT> {
  private _vpx = 0;
  private _vpy = 0;
  private _vpz = 0;

  constructor() {
    const native = new __PCLCore__.NormalEstimationPointXYZNormal();
    super(native);
  }

  public setViewPoint(vpx: number, vpy: number, vpz: number) {
    this._native.setViewPoint(vpx, vpy, vpz);
    this._vpx = vpx;
    this._vpy = vpy;
    this._vpz = vpz;
  }

  public getViewPoint() {
    return {
      vpx: this._vpx,
      vpy: this._vpy,
      vpz: this._vpz,
    };
  }

  public useSensorOriginAsViewPoint() {
    this._native.useSensorOriginAsViewPoint();
  }
}

export default NormalEstimation;
