import * as PCL from '../../';
import { getTestPCDFile } from '../utils';

describe('IterativeClosestPoint', () => {
  it('should to return a registered point cloud', () => {
    const bun0 = getTestPCDFile('bun0.pcd');
    const bun4 = getTestPCDFile('bun4.pcd');

    const source = PCL.loadPCDData<PCL.PointXYZ>(bun0, PCL.PointXYZ);
    const target = PCL.loadPCDData<PCL.PointXYZ>(bun4, PCL.PointXYZ);
    const final = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

    const icp = new PCL.IterativeClosestPoint<PCL.PointXYZ>();
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.setMaxCorrespondenceDistance(0.05);
    icp.align(final);

    expect(final.size).toBe(source.size);
  });
});
