import { PointCloud } from '@/modules/common/PointCloud';
import {
  PointXYZ,
  PointXYZI,
  PointXYZRGB,
  PointXYZRGBA,
  PointNormal,
} from '@/modules/common/point-types';

export function computeCloudResolution(
  cloud: PointCloud<
    PointXYZ | PointXYZI | PointXYZRGB | PointXYZRGBA | PointNormal
  >,
): number {
  const resolution = __PCLCore__[`computeCloudResolution${cloud._PT.name}`](
    cloud._native,
  );

  return resolution;
}
