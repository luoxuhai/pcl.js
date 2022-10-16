import { PointTypes, PointTypesTypeof, PointXYZ } from './point-types';
import { getPointType } from '@/utils';
import { Emscripten } from '@/types';
import Manager from './Manager';

export class Vector<T> {
  public manager = new Manager(this._native);

  constructor(public _native: Emscripten.NativeAPI) {}

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

class Indices extends Vector<number> {
  constructor(native?: Emscripten.NativeAPI) {
    const _native = native ?? new __PCLCore__.Indices();
    super(_native);
  }
}

class PCLHeader {
  public manager = new Manager(this._native);

  constructor(public _native: Emscripten.NativeAPI) {}

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

class PointIndices {
  public _native: Emscripten.NativeAPI;
  public manager: Manager;
  public header: PCLHeader;
  public indices: Indices;

  constructor(native?: Emscripten.NativeAPI) {
    this._native = native || new __PCLCore__.PointIndices();
    this.header = new PCLHeader(this._native.header);
    this.indices = new Indices(this._native.indices);
    this.manager = new Manager(this._native);
  }
}

class Points<T extends PointTypes> extends Vector<T> {
  private readonly _PT: PointTypesTypeof;

  constructor(_native: Emscripten.NativeAPI) {
    super(_native);
    this._PT = getPointType(_native, 'Points');
  }

  public get(index: number) {
    const PT = this._PT as any;
    const value = this._native.get(index);

    if (value instanceof __PCLCore__.FPFHSignature33) {
      return new PT(Array.from(value.getHistogram())) as T;
    } else {
      const args: number[] = Object.values(value);
      return new PT(...args) as T;
    }
  }
}

class PointCloud<T extends PointTypes = PointXYZ> {
  public _native: Emscripten.NativeAPI;
  public manager: Manager;
  private _points?: Points<T>;

  constructor(public readonly _PT: PointTypesTypeof = PointXYZ, _native?: Emscripten.NativeAPI) {
    this._native = _native ?? new __PCLCore__[`PointCloud${_PT.name}`]();
    this.manager = new Manager(this._native);
  }

  get isOrganized(): boolean {
    return this._native.isOrganized();
  }

  get isDense(): boolean {
    return this._native.is_dense;
  }

  set width(v: number) {
    this._native.width = v;
  }

  get width(): number {
    return this._native.width;
  }

  set height(v: number) {
    this._native.height = v;
  }

  get height(): number {
    return this._native.height;
  }

  get header() {
    return new PCLHeader(this._native.header);
  }

  get size(): number {
    return this.points.size;
  }

  get points() {
    if (!this._points) {
      this._points = wrapPoints(this._native.points);
    }

    return this._points;
  }

  /**
   * Removes all points in a cloud and sets the width and height to 0.
   */
  public clear() {
    this._native.clear();
  }

  /**
   * Resizes the container to contain `count` elements
   * @params count - New size of the point cloud
   * @params pt - The value to initialize the new points with
   */
  public resize(count: number, pt?: T) {
    this._native.resize(count, pt ?? new this._PT());
  }

  /**
   * Insert a new point in the cloud, at the end of the container.
   * @description This breaks the organized structure of the cloud by setting the height to 1!
   * @params pt - The point to insert
   */
  public addPoint(pt: T | null = null) {
    this._native.push_back(pt);
  }
}

function wrapPointCloud<T extends PointTypes>(_native: Emscripten.NativeAPI) {
  return new PointCloud<T>(getPointType(_native, 'PointCloud'), _native);
}

function wrapPoints<T extends PointTypes>(_native: Emscripten.NativeAPI) {
  return new Points<T>(_native);
}

export { PointCloud, Points, wrapPointCloud, wrapPoints, PointIndices, PCLHeader, Indices };
