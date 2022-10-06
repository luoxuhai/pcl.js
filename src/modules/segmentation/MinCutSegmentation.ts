import { PointCloud, wrapPointCloud, wrapPoints } from '@/modules/common/PointCloud';
import {
  XYZPointTypes,
  XYZPointTypesTypeof,
  PointXYZ,
  PointXYZRGB,
  Vector,
} from '@/modules/common/point-types';
import { UnionToIntersection, Emscripten } from '@/types';

class MinCutSegmentation<
  T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>,
> {
  public _native: Emscripten.NativeAPI;

  constructor(PT: XYZPointTypesTypeof = PointXYZ) {
    this._native = new __PCLCore__[`MinCutSegmentation${PT.name}`]();
  }

  public setInputCloud(cloud: PointCloud<T>) {
    this._native.setInputCloud(cloud._native);
  }

  public setSigma(sigma: number) {
    this._native.setSigma(sigma);
  }

  public getSigma(): number {
    return this._native.getSigma();
  }

  public setRadius(radius: number) {
    this._native.setRadius(radius);
  }

  public getRadius(): number {
    return this._native.getRadius();
  }

  public setSourceWeight(weight: number) {
    this._native.setSourceWeight(weight);
  }

  public getSourceWeight(): number {
    return this._native.getSourceWeight();
  }

  public setSearchMethod(tree: string | null) {
    this._native.setSearchMethod(tree);
  }

  public getSearchMethod(): string | null {
    return this._native.getSearchMethod();
  }

  public setNumberOfNeighbours(neighbourNumber: number) {
    this._native.setNumberOfNeighbours(neighbourNumber);
  }

  public getNumberOfNeighbours(): number {
    return this._native.getNumberOfNeighbours();
  }

  public setForegroundPoints(cloud: PointCloud<T>) {
    this._native.setForegroundPoints(cloud._native);
  }

  public getForegroundPoints() {
    return wrapPoints<T>(this._native.getForegroundPoints());
  }

  public setBackgroundPoints(cloud: PointCloud<T>) {
    this._native.setBackgroundPoints(cloud._native);
  }

  public getBackgroundPoints() {
    return wrapPoints<T>(this._native.getBackgroundPoints());
  }

  public extract() {
    const clusters = new Vector(new __PCLCore__.VectorPointIndices());
    this._native.extract(clusters._native);
    return clusters;
  }

  public getMaxFlow(): number {
    return this._native.getMaxFlow();
  }

  public getColoredCloud() {
    return wrapPointCloud<PointXYZRGB>(this._native.getColoredCloud());
  }
}

export default MinCutSegmentation;
