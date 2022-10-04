export class PointXY {
  constructor(public x = 0, public y = 0) {}
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

export class PointXYZL extends PointXYZ {
  constructor(x?: number, y?: number, z?: number, public label = 0) {
    super(x, y, z);
  }
}

export class InterestPoint extends PointXYZ {
  constructor(x?: number, y?: number, z?: number, public strength = 0) {
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

export class PointXYZRGBL extends PointXYZRGB {
  constructor(
    x?: number,
    y?: number,
    z?: number,
    r?: number,
    g?: number,
    b?: number,
    public label = 0,
  ) {
    super(x, y, z, r, g, b);
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

export class PointXYZLNormal extends PointNormal {
  constructor(
    x?: number,
    y?: number,
    z?: number,
    public label = 0,
    normalX?: number,
    normalY?: number,
    normalZ?: number,
    curvature?: number,
  ) {
    super(x, y, z, normalX, normalY, normalZ, curvature);
  }
}

export class PointXYZINormal extends PointNormal {
  constructor(
    x?: number,
    y?: number,
    z?: number,
    public intensity = 0,
    normalX?: number,
    normalY?: number,
    normalZ?: number,
    curvature?: number,
  ) {
    super(x, y, z, normalX, normalY, normalZ, curvature);
  }
}

export class PointXYZRGBNormal extends PointNormal {
  constructor(
    x?: number,
    y?: number,
    z?: number,
    public r = 0,
    public g = 0,
    public b = 0,
    normalX?: number,
    normalY?: number,
    normalZ?: number,
    curvature?: number,
  ) {
    super(x, y, z, normalX, normalY, normalZ, curvature);
  }
}

export class PointSurfel extends PointXYZ {
  constructor(
    x?: number,
    y?: number,
    z?: number,
    public normalX = 0,
    public normalY = 0,
    public normalZ = 0,
    public r = 0,
    public g = 0,
    public b = 0,
    public a = 0,
    public radius = 0,
    public confidence = 0,
    public curvature = 0,
  ) {
    super(x, y, z);
  }
}

export type PointTypesUnion =
  | PointXY
  | PointXYZ
  | PointXYZI
  | InterestPoint
  | PointXYZL
  | PointXYZRGB
  | PointXYZRGBA
  | PointXYZRGBL
  | Normal
  | PointNormal
  | PointXYZRGBNormal
  | PointXYZINormal
  | PointXYZLNormal
  | PointSurfel;

export type TPointTypesUnion =
  | typeof PointXY
  | typeof PointXYZ
  | typeof PointXYZI
  | typeof InterestPoint
  | typeof PointXYZL
  | typeof PointXYZRGB
  | typeof PointXYZRGBA
  | typeof PointXYZRGBL
  | typeof Normal
  | typeof PointNormal
  | typeof PointXYZRGBNormal
  | typeof PointXYZINormal
  | typeof PointXYZLNormal
  | typeof PointSurfel;

export type PointTypesIntersection = UnionToIntersection<PointTypesUnion>;

export type TPointTypesIntersection = UnionToIntersection<TPointTypesUnion>;

export const pointTypeMap = {
  PointXY,
  PointXYZ,
  PointXYZI,
  InterestPoint,
  PointXYZL,
  PointXYZRGB,
  PointXYZRGBA,
  PointXYZRGBL,
  Normal,
  PointNormal,
  PointXYZRGBNormal,
  PointXYZINormal,
  PointXYZLNormal,
  PointSurfel,
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
