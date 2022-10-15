import Feature from './Feature';
import { PointCloud } from '@/modules/common/PointCloud';
import { XYZPointTypes, Normal, FPFHSignature33 } from '@/modules/common/point-types';
import { Emscripten } from '@/types';

class FeatureFromNormals<
  PointInT extends XYZPointTypes,
  PointNT extends Normal = Normal,
  PointOutT extends FPFHSignature33 = FPFHSignature33,
> extends Feature<PointInT, PointOutT> {
  private _normals?: PointCloud<PointNT>;

  constructor(native: Emscripten.NativeAPI) {
    super(native);
  }

  public setInputNormals(normals: PointCloud<PointNT>) {
    this._native.setInputNormals(normals._native);
    this._normals = normals;
  }

  public getInputNormals() {
    return this._normals;
  }
}

export default FeatureFromNormals;
