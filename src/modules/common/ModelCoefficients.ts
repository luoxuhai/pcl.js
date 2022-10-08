import { Emscripten } from '@/types';
import { PCLHeader, Vector } from '@/modules/common/PointCloud';

class ModelCoefficients {
  public _native: Emscripten.NativeAPI;

  constructor() {
    this._native = new __PCLCore__.ModelCoefficients();
  }

  get header() {
    return new PCLHeader(this._native.header);
  }

  get values() {
    return new Vector(this._native.values);
  }
}

export default ModelCoefficients;
