import { PointCloud } from '@/modules/common/PointCloud';
import { XYZPointTypes } from '@/modules/common/point-types';

export function computeCloudResolution(
  cloud: PointCloud<XYZPointTypes>,
): number {
  const resolution = __PCLCore__[`computeCloudResolution${cloud._PT.name}`](
    cloud._native,
  );

  return resolution;
}
