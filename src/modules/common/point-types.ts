export class Point {
  constructor(public x: number, public y: number) {}
}

export class PointXY extends Point {
  constructor(x = 0, y = 0) {
    super(x, y);
  }
}

export class PointXYZ extends PointXY {
  constructor(x = 0, y = 0, public z = 0) {
    super(x, y);
  }
}

export class PointXYZI extends PointXYZ {
  constructor(x = 0, y = 0, z = 0, public intensity = 0) {
    super(x, y, z);
  }
}

export class PointXYZRGB extends PointXYZ {
  constructor(x = 0, y = 0, z = 0, public r = 0, public g = 0, public b = 0) {
    super(x, y, z);
  }
}

export class PointXYZRGBA extends PointXYZ {
  constructor(
    x = 0,
    y = 0,
    z = 0,
    public r = 0,
    public g = 0,
    public b = 0,
    public a = 0,
  ) {
    super(x, y, z);
  }
}

export class Normal {
  constructor(
    public normalX = 0,
    public normalY = 0,
    public normalZ = 0,
    public curvature = 0,
  ) {}
}

export class PointNormal extends PointXYZ {
  constructor(
    x = 0,
    y = 0,
    z = 0,
    public normalX = 0,
    public normalY = 0,
    public normalZ = 0,
    public curvature = 0,
  ) {
    super(x, y, z);
  }
}

export type PointTypesUnion =
  | PointXYZ
  | PointXYZI
  | PointXYZRGB
  | PointXYZRGBA
  | Normal
  | PointNormal;

export type TPointTypesUnion =
  | typeof PointXYZ
  | typeof PointXYZI
  | typeof PointXYZRGB
  | typeof PointXYZRGBA
  | typeof Normal
  | typeof PointNormal;

export type PointTypesIntersection = PointXYZ &
  PointXYZI &
  PointXYZRGB &
  PointXYZRGBA &
  Normal &
  PointNormal;

export type TPointTypesIntersection = typeof PointXYZ &
  typeof PointXYZI &
  typeof PointXYZRGB &
  typeof PointXYZRGBA &
  typeof Normal &
  typeof PointNormal;

export const pointTypeMap = {
  PointXYZ,
  PointXYZI,
  PointXYZRGB,
  PointXYZRGBA,
  Normal,
  PointNormal,
};

export abstract class NativeObject {
  abstract _native: Emscripten.NativeAPI;

  public clone() {
    return this._native.clone();
  }

  public delete() {
    return this._native.delete();
  }

  public deleteLater() {
    return this._native.deleteLater();
  }

  public isDeleted() {
    return this._native.isDeleted();
  }
}

export class Vector<T> extends NativeObject {
  constructor(public _native: Emscripten.NativeAPI) {
    super();
  }

  get size(): number {
    return this._native.size();
  }

  public set(index: number, value: T): boolean {
    return this._native.set(index, value);
  }

  public get(index: number): T {
    return this._native.get(index);
  }

  public push(value: T) {
    this._native.push_back(value);
  }

  public isEmpty(): boolean {
    return this._native.empty();
  }

  public resize(count: number, value?: T) {
    this._native.resize(count, value ?? null);
  }

  public clear() {
    this._native.clear();
  }
}
