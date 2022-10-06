import { PointCloud, Indices } from '@/modules/common/PointCloud';
import {
  XYZPointTypes,
  NormalPointTypes,
  PointNormal,
  PointXYZ,
} from '@/modules/common/point-types';
import { UnionToIntersection } from '@/types/utils';

function removeNaNFromPointCloud<
  T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>,
>(cloudIn: PointCloud<T>, cloudOut?: PointCloud<T>, indices?: Indices) {
  const _cloudOut = cloudOut ?? new PointCloud<T>(cloudIn._PT);
  const _indices = indices ?? new Indices();

  __PCLCore__[`removeNaNFromPointCloud${cloudIn._PT.name}`](
    cloudIn._native,
    _cloudOut._native,
    _indices._native,
  );

  return { cloud: _cloudOut, indices: _indices };
}

function removeNaNNormalsFromPointCloud<
  T extends NormalPointTypes = PointNormal & Partial<UnionToIntersection<NormalPointTypes>>,
>(cloudIn: PointCloud<T>, cloudOut?: PointCloud<T>, indices?: Indices) {
  const _cloudOut = cloudOut ?? new PointCloud<T>(cloudIn._PT);
  const _indices = indices ?? new Indices();

  __PCLCore__[`removeNaNNormalsFromPointCloud${cloudIn._PT.name}`](
    cloudIn._native,
    _cloudOut._native,
    _indices._native,
  );

  return { cloud: _cloudOut, indices: _indices };
}

export { removeNaNFromPointCloud, removeNaNNormalsFromPointCloud };
