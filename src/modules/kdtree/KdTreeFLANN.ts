import KdTree from './KdTree';
import { Indices } from '@/modules/common/PointCloud';
import {
  XYZPointTypes,
  XYZPointTypesTypeof,
  PointXYZ,
  Vector,
} from '@/modules/common/point-types';

class KdTreeFLANN<
  T extends XYZPointTypes = PointXYZ &
    Partial<UnionToIntersection<XYZPointTypes>>,
> extends KdTree<T> {
  constructor(_PT: XYZPointTypesTypeof = PointXYZ, sorted = true) {
    super(new __PCLCore__[`KdTreeFLANN${_PT.name}`](sorted));
  }

  public setSortedResults(sorted: boolean) {
    this._native.setSortedResults(sorted);
  }

  public setEpsilon(eps: number) {
    this._native.setEpsilon(eps);
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
