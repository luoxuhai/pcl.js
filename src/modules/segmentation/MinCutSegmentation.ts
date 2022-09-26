import {
  PointCloud,
  PointXYZ,
  PointTypesIntersection,
  PointTypesUnion,
  TPointTypesUnion,
  Vector,
} from '../point-types';

class MinCutSegmentation<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> {
  public native: Emscripten.NativeAPI;

  constructor(PT: TPointTypesUnion = PointXYZ) {
    this.native = new __PCLCore__[`MinCutSegmentation${PT.name}`]();
  }

  public setInputCloud(cloud: PointCloud<T>) {
    return this.native.setInputCloud(cloud.native);
  }

  public setSigma(sigma: number): void {
    return this.native.setSigma(sigma) as void;
  }

  public getSigma(): number {
    return this.native.getSigma() as number;
  }

  public setRadius(radius: number): void {
    return this.native.setRadius(radius) as void;
  }

  public getRadius(): number {
    return this.native.getRadius() as number;
  }

  public setSourceWeight(weight: number): void {
    return this.native.setSourceWeight(weight) as void;
  }

  public getSourceWeight(): number {
    return this.native.getSourceWeight() as number;
  }

  public setSearchMethod(tree: string | null): void {
    return this.native.setSearchMethod(tree) as void;
  }

  public getSearchMethod(): string | null {
    return this.native.getSearchMethod() as string | null;
  }

  public setNumberOfNeighbours(neighbourNumber: number): void {
    return this.native.setNumberOfNeighbours(neighbourNumber) as void;
  }

  public getNumberOfNeighbours(): number {
    return this.native.getNumberOfNeighbours() as number;
  }

  public setForegroundPoints(foregroundPoints: PointCloud<T>): void {
    return this.native.setForegroundPoints(foregroundPoints) as void;
  }

  public getForegroundPoints(): PointCloud<T> {
    return this.native.getForegroundPoints() as PointCloud<T>;
  }

  public setBackgroundPoints(backgroundPoints: PointCloud<T>): void {
    return this.native.setBackgroundPoints(backgroundPoints) as void;
  }

  public getBackgroundPoints(): PointCloud<T> {
    return this.native.getBackgroundPoints() as PointCloud<T>;
  }

  // Vector<PointIndices>
  public extract(clusters: Vector<Vector<number>>): void {
    return this.native.extract(clusters) as void;
  }

  public getMaxFlow(): number {
    return this.native.getMaxFlow() as number;
  }

  // mGraphPtr
  // public getGraph(): ??? {
  //   return this.native.getGraph() as ???;
  // }

  public getColoredCloud(): PointCloud<T> {
    return this.native.getColoredCloud() as PointCloud<T>;
  }
}

export default MinCutSegmentation;
