import {
  PointTypesUnion,
  TPointTypesUnion,
  PointTypesIntersection,
  pointTypeMap,
  Points,
  PointXYZ,
  PointXYZI,
} from './type';

class PointCloud<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> {
  public native: Emscripten.NativeAPI;
  public readonly PT: TPointTypesUnion;

  constructor(PT: TPointTypesUnion = PointXYZ, native?: Emscripten.NativeAPI) {
    this.PT = PT;
    this.native =
      native ?? new __PCLCore__[`PointCloud${PT.name}`]().makeShared();
  }

  public isOrganized(): boolean {
    return this.native.isOrganized();
  }

  get isDense(): boolean {
    return this.native.is_dense;
  }

  get points(): Points<T> {
    return this.native.points;
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

function wrapPointCloud<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
>(native: Emscripten.NativeAPI) {
  const pointType = native.$$.ptrType.registeredClass.name.replace(
    'PointCloud',
    '',
  ) as string;
  return new PointCloud<T>(pointTypeMap[pointType], native);
}

export { PointCloud, wrapPointCloud };
export * from './type';
