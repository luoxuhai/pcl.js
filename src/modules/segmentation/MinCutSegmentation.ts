import { PointCloud } from '@/modules/common/PointCloud';
import {
  PointXYZ,
  PointTypesIntersection,
  PointTypesUnion,
  TPointTypesUnion,
  Vector,
} from '@/modules/common/point-types';

class MinCutSegmentation<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> {
  public _native: Emscripten.NativeAPI;

  constructor(PT: TPointTypesUnion = PointXYZ) {
    this._native = new __PCLCore__[`MinCutSegmentation${PT.name}`]();
  }

  public setInputCloud(cloud: PointCloud<T>) {
    return this._native.setInputCloud(cloud._native);
  }

  public setSigma(sigma: number): void {
    return this._native.setSigma(sigma) as void;
  }

  public getSigma(): number {
    return this._native.getSigma() as number;
  }

  public setRadius(radius: number): void {
    return this._native.setRadius(radius) as void;
  }

  public getRadius(): number {
    return this._native.getRadius() as number;
  }

  public setSourceWeight(weight: number): void {
    return this._native.setSourceWeight(weight) as void;
  }

  public getSourceWeight(): number {
    return this._native.getSourceWeight() as number;
  }

  public setSearchMethod(tree: string | null): void {
    return this._native.setSearchMethod(tree) as void;
  }

  public getSearchMethod(): string | null {
    return this._native.getSearchMethod() as string | null;
  }

  public setNumberOfNeighbours(neighbourNumber: number): void {
    return this._native.setNumberOfNeighbours(neighbourNumber) as void;
  }

  public getNumberOfNeighbours(): number {
    return this._native.getNumberOfNeighbours() as number;
  }

  public setForegroundPoints(cloud: PointCloud<T>) {
    return this._native.setForegroundPoints(cloud._native);
  }

  public getForegroundPoints(): PointCloud<T> {
    return this._native.getForegroundPoints() as PointCloud<T>;
  }

  public setBackgroundPoints(cloud: PointCloud<T>) {
    return this._native.setBackgroundPoints(cloud._native);
  }

  public getBackgroundPoints(): PointCloud<T> {
    return this._native.getBackgroundPoints() as PointCloud<T>;
  }

  public extract() {
    const clusters = new Vector(new __PCLCore__.VectorPointIndices());
    this._native.extract(clusters._native);
    return clusters;
  }

  public getMaxFlow(): number {
    return this._native.getMaxFlow() as number;
  }

  public getColoredCloud(): PointCloud<T> {
    return this._native.getColoredCloud() as PointCloud<T>;
  }
}

export default MinCutSegmentation;
