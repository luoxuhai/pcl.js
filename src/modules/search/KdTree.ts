import Search from './Search';
import { PointCloud, Indices } from '@/modules/common/PointCloud';
import {
  PointTypesUnion,
  PointTypesIntersection,
  TPointTypesUnion,
  PointXYZ,
  Vector,
} from '@/modules/common/point-types';

class KdTree<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends Search<T> {
  constructor(_PT: TPointTypesUnion = PointXYZ, sorted = true) {
    super(new __PCLCore__[`SearchKdTree${_PT.name}`](sorted));
  }

  public setSortedResults(sorted: boolean) {
    this._native.setSortedResults(sorted);
  }

  public setEpsilon(eps: number) {
    this._native.setEpsilon(eps);
  }

  public setInputCloud(cloud: PointCloud<T>) {
    this._native.setInputCloud(cloud._native);
  }

  public nearestKSearch(point: PointTypesUnion, k: number) {
    const indices = new Indices();
    const distances = new Vector<number>(new __PCLCore__.VectorFloat());
    this._native.nearestKSearch(point, k, indices._native, distances._native);

    return {
      indices,
      distances,
    };
  }

  public radiusSearch(point: PointTypesUnion, radius: number, maxNN = 0) {
    const indices = new Indices();
    const distances = new Vector<number>(new __PCLCore__.VectorFloat());
    this._native.radiusSearch(
      point,
      radius,
      indices._native,
      distances._native,
      maxNN,
    );

    return {
      indices,
      distances,
    };
  }
}

export default KdTree;
