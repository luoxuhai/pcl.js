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

  get size() {
    return this._native.size();
  }

  public set(index: number, value: T) {
    return this._native.set(index, value);
  }

  public get(index: number): T {
    return this._native.get(index);
  }

  public push(value: T) {
    this._native.push_back(value);
  }

  public isEmpty() {
    return this._native.empty();
  }

  public resize(count: number, value?: T) {
    return this._native.resize(count, value ?? null);
  }

  public clear() {
    return this._native.clear();
  }
}

export class Indices extends Vector<number> {
  constructor(native?: Emscripten.NativeAPI) {
    const _native = native ?? new __PCLCore__.Indices();
    super(_native);
  }
}

export class PCLHeader extends NativeObject {
  constructor(public _native: Emscripten.NativeAPI) {
    super();
  }

  get seq(): number {
    return this._native.seq;
  }

  get stamp(): bigint {
    return this._native.stamp;
  }

  get frameId(): string {
    return this._native.frame_id;
  }
}

export class PointIndices extends NativeObject {
  public header: PCLHeader;
  public indices: Indices;

  constructor(public _native: Emscripten.NativeAPI) {
    super();
    this.header = new PCLHeader(this._native.header);
    this.indices = new Indices(this._native.indices);
  }
}

// export class PointIndices {
//   constructor(public indices: Vector<number>) {}
// }

// export class vectorPointIndices {
//   constructor(public vectorPointIndices: Vector<PointIndices>) {}
// }

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
