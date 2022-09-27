import KdTree from './KdTree';
import {
  PointTypesUnion,
  PointTypesIntersection,
  TPointTypesUnion,
  PointXYZ,
  Indices,
  Vector,
} from '../point-types';

class KdTreeFLANN<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends KdTree<T> {
  constructor(_PT: TPointTypesUnion = PointXYZ, sorted = true) {
    super(new __PCLCore__[`KdTreeFLANN${_PT.name}`](sorted));
  }

  public setSortedResults(sorted: boolean) {
    this.native.setSortedResults(sorted);
  }

  public setEpsilon(eps: number) {
    this.native.setEpsilon(eps);
  }

  public nearestKSearch(point: PointTypesUnion, k: number) {
    const indices = new Indices();
    const distances = new Vector<number>(new __PCLCore__.VectorFloat());
    this.native.nearestKSearch(point, k, indices.native, distances.native);

    return {
      indices,
      distances,
    };
  }

  public radiusSearch(point: PointTypesUnion, radius: number, maxNN = 0) {
    const indices = new Indices();
    const distances = new Vector<number>(new __PCLCore__.VectorFloat());
    this.native.radiusSearch(
      point,
      radius,
      indices.native,
      distances.native,
      maxNN,
    );

    return {
      indices,
      distances,
    };
  }
}

export default KdTreeFLANN;
