import { PointCloud, Indices } from '@/modules/common/PointCloud';
import {
  PointTypesUnion,
  PointNormal,
  Normal,
} from '@/modules/common/point-types';

function removeNaNFromPointCloud<T extends Partial<PointTypesUnion>>(
  cloudIn: PointCloud<T>,
  cloudOut?: PointCloud<T>,
  indices?: Indices,
) {
  const _cloudOut = cloudOut ?? new PointCloud<T>(cloudIn._PT);
  const _indices = indices ?? new Indices();

  __PCLCore__[`removeNaNFromPointCloud${cloudIn._PT.name}`](
    cloudIn._native,
    _cloudOut._native,
    _indices._native,
  );

  return { cloud: _cloudOut, indices: _indices };
}

function removeNaNNormalsFromPointCloud<T extends Normal | PointNormal>(
  cloudIn: PointCloud<T>,
  cloudOut?: PointCloud<T>,
  indices?: Indices,
) {
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
