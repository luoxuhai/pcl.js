export class PointXY {
  // HACK: To distinguish between different types
  private readonly _PointXY = 'PointXY';
  constructor(public x = 0, public y = 0) {}
}

export class PointXYZ {
  private readonly _PointXYZ = 'PointXYZ';
  constructor(public x = 0, public y = 0, public z = 0) {}
}

export class PointXYZI {
  private readonly _PointXYZI = 'PointXYZI';

  constructor(public x = 0, public y = 0, public z = 0, public intensity = 0) {}
}

export class PointXYZL {
  private readonly _PointXYZL = 'PointXYZL';
  constructor(public x = 0, public y = 0, public z = 0, public label = 0) {}
}

export class InterestPoint {
  private readonly _InterestPoint = 'InterestPoint';
  constructor(public x = 0, public y = 0, public z = 0, public strength = 0) {}
}

export class PointXYZRGB {
  private readonly _PointXYZRGB = 'PointXYZRGB';
  constructor(
    public x = 0,
    public y = 0,
    public z = 0,
    public r = 0,
    public g = 0,
    public b = 0,
  ) {}
}

export class PointXYZRGBA {
  private readonly _PointXYZRGBA = 'PointXYZRGBA';
  constructor(
    public x = 0,
    public y = 0,
    public z = 0,
    public r = 0,
    public g = 0,
    public b = 0,
    public a = 0,
  ) {}
}

export class PointXYZRGBL {
  private readonly _PointXYZRGBL = 'PointXYZRGBL';
  constructor(
    public x = 0,
    public y = 0,
    public z = 0,
    public r = 0,
    public g = 0,
    public b = 0,
    public label = 0,
  ) {}
}

export class Normal {
  private readonly _Normal = 'Normal';
  constructor(
    public normalX = 0,
    public normalY = 0,
    public normalZ = 0,
    public curvature = 0,
  ) {}
}

export class PointNormal {
  private readonly _PointNormal = 'PointNormal';
  constructor(
    public x = 0,
    public y = 0,
    public z = 0,
    public normalX = 0,
    public normalY = 0,
    public normalZ = 0,
    public curvature = 0,
  ) {}
}

export class PointXYZLNormal {
  private readonly _PointXYZLNormal = 'PointXYZLNormal';
  constructor(
    public x = 0,
    public y = 0,
    public z = 0,
    public label = 0,
    public normalX = 0,
    public normalY = 0,
    public normalZ = 0,
    public curvature = 0,
  ) {}
}

export class PointXYZINormal {
  private readonly _PointXYZINormal = 'PointXYZINormal';
  constructor(
    public x = 0,
    public y = 0,
    public z = 0,
    public intensity = 0,
    public normalX = 0,
    public normalY = 0,
    public normalZ = 0,
    public curvature = 0,
  ) {}
}

export class PointXYZRGBNormal {
  private readonly _PointXYZRGBNormal = 'PointXYZRGBNormal';
  constructor(
    public x = 0,
    public y = 0,
    public z = 0,
    public r = 0,
    public g = 0,
    public b = 0,
    public normalX = 0,
    public normalY = 0,
    public normalZ = 0,
    public curvature = 0,
  ) {}
}

export class PointSurfel {
  private readonly _PointSurfel = 'PointSurfel';
  constructor(
    public x = 0,
    public y = 0,
    public z = 0,
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
  ) {}
}

// Define all point types
export type PointTypes =
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

export type PointTypesTypeof =
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

// Define all point types that include XYZ data
export type XYZPointTypes =
  | PointXYZ
  | PointXYZI
  | PointXYZL
  | PointXYZRGBA
  | PointXYZRGB
  | PointXYZRGBL
  | InterestPoint
  | PointNormal
  | PointXYZRGBNormal
  | PointXYZINormal
  | PointXYZLNormal
  | PointSurfel;

export type XYZPointTypesTypeof =
  | typeof PointXYZ
  | typeof PointXYZI
  | typeof PointXYZL
  | typeof PointXYZRGBA
  | typeof PointXYZRGB
  | typeof PointXYZRGBL
  | typeof InterestPoint
  | typeof PointNormal
  | typeof PointXYZRGBNormal
  | typeof PointXYZINormal
  | typeof PointXYZLNormal
  | typeof PointSurfel;

// Define all point types with XYZ and label
export type XYZLPointTypes = PointXYZL | PointXYZRGBL | PointXYZLNormal;

// Define all point types that include RGB data
export type RGBPointTypes =
  | PointXYZRGB
  | PointXYZRGBA
  | PointXYZRGBL
  | PointXYZRGBNormal
  | PointSurfel;

// Define all point types that include normal data
export type NormalPointTypes =
  | Normal
  | PointNormal
  | PointXYZRGBNormal
  | PointXYZINormal
  | PointXYZLNormal
  | PointSurfel;

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
