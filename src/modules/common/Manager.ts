import { Emscripten } from '@/types';

class Manager {
  constructor(private _native: Emscripten.NativeAPI) {}

  /**
   * Delete a C ++ object
   * {@link https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html?highlight=clone#memory-management}
   */
  public delete() {
    this._native.delete();
  }

  public isDeleted() {
    return this._native.isDeleted();
  }

  /**
   * {@link https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html?highlight=clone#cloning-and-reference-counting}
   * @returns A new handle
   */
  public clone() {
    return this._native.clone();
  }
}

export default Manager;
