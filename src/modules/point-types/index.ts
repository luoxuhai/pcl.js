import {
  PointTypesUnion,
  TPointTypesUnion,
  PointTypesIntersection,
  PointXYZ,
  Vector,
} from './type';
import { getPointType } from '../../utils';

class PointCloud<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> {
  public native: Emscripten.NativeAPI;

  constructor(
    public readonly PT: TPointTypesUnion = PointXYZ,
    native?: Emscripten.NativeAPI,
  ) {
    this.native = native ?? new __PCLCore__[`PointCloud${PT.name}`]();
  }

  public isOrganized(): boolean {
    return this.native.isOrganized();
  }

  get isDense(): boolean {
    return this.native.is_dense;
  }

  get points(): Points<T> {
    return wrapPoints(this.native.points);
  }

  set width(v: number) {
    this.native.width = v;
  }

  get width(): number {
    return this.native.width;
  }

  set height(v: number) {
    this.native.height = v;
  }

  get height(): number {
    return this.native.height;
  }

  /**
   * Removes all points in a cloud and sets the width and height to 0.
   */
  public clear() {
    this.native.clear();
  }

  /**
   * Resizes the container to contain `count` elements
   * @params count - New size of the point cloud
   * @params pt - The value to initialize the new points with
   */
  public resize(count: number, pt?: T) {
    this.native.resize(count, pt ?? new this.PT());
  }

  /**
   * Insert a new point in the cloud, at the end of the container.
   * @description This breaks the organized structure of the cloud by setting the height to 1!
   * @params pt - The point to insert
   */
  public push(pt: T | null = null) {
    this.native.push_back(pt);
  }

  public delete() {
    this.native.delete();
  }
}

class Points<T> {
  private readonly PT: TPointTypesUnion;

  constructor(public native: Vector<T>) {
    this.PT = getPointType(native, 'Points');
  }

  public set(index: number, value: T) {
    return this.native.set(index, value);
  }

  public get(index: number) {
    const args: number[] = Object.values(this.native.get(index) ?? {});
    return new this.PT(...args) as unknown as T;
  }

  public push(value: T) {
    this.native.push_back(value);
  }

  public size() {
    return this.native.size();
  }

  public empty() {
    return this.native.empty();
  }

  public clear() {
    return this.native.clear();
  }
}

function wrapPointCloud<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
>(native: Emscripten.NativeAPI) {
  return new PointCloud<T>(getPointType(native, 'PointCloud'), native);
}

function wrapPoints<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
>(native: Vector<T>) {
  return new Points<T>(native);
}

export { PointCloud, Points, wrapPointCloud, wrapPoints };
export * from './type';
