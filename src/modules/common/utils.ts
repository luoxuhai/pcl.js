import { PointCloud, PointXYZ } from '../point-types';

export function computeCloudResolution(cloud: PointCloud<PointXYZ>): number {
  return __PCLCore__.computeCloudResolution(cloud.native);
}
