import Search from './Search';
import { PointCloud, Indices, Vector } from '@/modules/common/PointCloud';
import { XYZPointTypes, XYZPointTypesTypeof, PointXYZ } from '@/modules/common/point-types';
import { UnionToIntersection } from '@/types/utils';

class KdTree<
  T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>,
> extends Search<T> {
  constructor(_PT: XYZPointTypesTypeof = PointXYZ, sorted = true) {
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

  public nearestKSearch(point: T, k: number) {
    const indices = new Indices();
    const distances = new Vector<number>(new __PCLCore__.VectorFloat());
    this._native.nearestKSearch(point, k, indices._native, distances._native);

    return {
      indices,
      distances,
    };
  }

  public radiusSearch(point: T, radius: number, maxNN = 0) {
    const indices = new Indices();
    const distances = new Vector<number>(new __PCLCore__.VectorFloat());
    this._native.radiusSearch(point, radius, indices._native, distances._native, maxNN);

    return {
      indices,
      distances,
    };
  }
}

export default KdTree;
