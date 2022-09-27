export class Point {
  constructor(public x: number, public y: number, public z: number) {}
}

export class PointXYZ extends Point {
  constructor(x = 0, y = 0, z = 0) {
    super(x, y, z);
  }
}

export class PointXYZI extends Point {
  constructor(x = 0, y = 0, z = 0, public intensity = 0) {
    super(x, y, z);
  }
}

export class PointXYZRGB extends Point {
  constructor(x = 0, y = 0, z = 0, public rgb = 0) {
    super(x, y, z);
  }
}
export class PointXYZRGBA extends Point {
  constructor(x = 0, y = 0, z = 0, public rgba = 0) {
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

export class PointNormal extends Point {
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

export class Vector<T> {
  constructor(public native: Emscripten.NativeAPI) {}

  get size() {
    return this.native.size();
  }

  public set(index: number, value: T) {
    return this.native.set(index, value);
  }

  public get(index: number): T {
    return this.native.get(index);
  }

  public push(value: T) {
    this.native.push_back(value);
  }

  public isEmpty() {
    return this.native.empty();
  }

  public resize(count: number, value?: T) {
    return this.native.resize(count, value ?? null);
  }

  public clear() {
    return this.native.clear();
  }

  public delete() {
    return this.native.delete();
  }
}

export class Indices extends Vector<number> {
  constructor(native?: Emscripten.NativeAPI) {
    const _native = native ?? new __PCLCore__.Indices();
    super(_native);
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
