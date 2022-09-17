import KdTree from './KdTree';
import {
  PointTypesUnion,
  PointTypesIntersection,
  TPointTypesUnion,
  PointXYZ,
} from '../point-types';

class KdTreeFLANN<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends KdTree<T> {
  constructor(PT: TPointTypesUnion = PointXYZ, sorted = true) {
    super(new __PCLCore__[`KdTreeFLANN${PT.name}`](sorted));
  }

  public setSortedResults(sorted: boolean) {
    this.native.setSortedResults(sorted);
  }

  public setEpsilon(eps: number) {
    this.native.setEpsilon(eps);
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

export default KdTreeFLANN;
