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

export interface Vector<T> extends Emscripten.NativeAPI {
  get(index: number): T;
  set(index: number, value: T): boolean;
  push_back(value: T): void;
  size(): number;
  empty(): boolean;
  clear(): boolean;
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
