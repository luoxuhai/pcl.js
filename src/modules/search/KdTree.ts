import Search from './Search';
import {
  PointTypesUnion,
  PointTypesIntersection,
  TPointTypesUnion,
  PointXYZ,
  PointCloud,
} from '../point-types';

class KdTree<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends Search<T> {
  constructor(PT: TPointTypesUnion = PointXYZ, sorted = true) {
    super(new __PCLCore__[`SearchKdTree${PT.name}`](sorted));
  }

  public setSortedResults(sorted: boolean) {
    this.native.setSortedResults(sorted);
  }

  public setEpsilon(eps: number) {
    this.native.setEpsilon(eps);
  }

  public setInputCloud(cloud: PointCloud<T>) {
    this.native.setInputCloud(cloud.native);
  }

  public nearestKSearch(point: PointTypesUnion, k: number) {
    const indices = new __PCLCore__.Indices();
    const distances = new __PCLCore__.VectorFloat();
    this.native.nearestKSearch(point, k, indices, distances);

    return {
      indices,
      distances,
    };
  }

  public radiusSearch(point: PointTypesUnion, radius: number, maxNN = 0) {
    const indices = new __PCLCore__.Indices();
    const distances = new __PCLCore__.VectorFloat();
    this.native.radiusSearch(point, radius, indices, distances, maxNN);

    return {
      indices,
      distances,
    };
  }
}

export default KdTree;
