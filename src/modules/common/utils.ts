import { PointCloud } from '@/modules/common/PointCloud';
import { XYZPointTypes, PointXYZ } from '@/modules/common/point-types';

export function computeCloudResolution(cloud: PointCloud<XYZPointTypes>): number {
  const resolution = __PCLCore__[`computeCloudResolution${cloud._PT.name}`](cloud._native);

  return resolution;
}

export function toXYZPointCloud(
  cloudIn: PointCloud<XYZPointTypes>,
  cloudOut: PointCloud<PointXYZ>,
) {
  __PCLCore__[`toXYZPointCloud${cloudIn._PT.name}`](cloudIn._native, cloudOut._native);
}
