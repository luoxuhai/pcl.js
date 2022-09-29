import {
  PointCloud,
  PointXYZ,
  PointXYZI,
  PointXYZRGB,
  PointXYZRGBA,
  PointNormal,
} from '../point-types';

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
