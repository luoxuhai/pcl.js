export interface Vector<T> extends Emscripten.NativeAPI {
  get(index: number): T;
  set(index: number, value: T): boolean;
  push_back(value: T): void;
  size(): number;
  empty(): boolean;
  clear(): boolean;
}

export class PointIndices {
  constructor(public x: Vector<number>) {}
}
