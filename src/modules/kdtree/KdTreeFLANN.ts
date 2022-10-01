import KdTree from './KdTree';
import { Indices } from '@/modules/common/PointCloud';
import {
  PointTypesUnion,
  PointTypesIntersection,
  TPointTypesUnion,
  PointXYZ,
  Vector,
} from '@/modules/common/point-types';

class KdTreeFLANN<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends KdTree<T> {
  constructor(_PT: TPointTypesUnion = PointXYZ, sorted = true) {
    super(new __PCLCore__[`KdTreeFLANN${_PT.name}`](sorted));
  }

  public setSortedResults(sorted: boolean) {
    this._native.setSortedResults(sorted);
  }

  public setEpsilon(eps: number) {
    this._native.setEpsilon(eps);
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

export default KdTreeFLANN;
