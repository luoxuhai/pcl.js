import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('ISSKeypoint3D', () => {
  it('should compute the ISS 3D keypoints - Without Boundary Estimation', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const data = fs.readFileSync(path.join(global.ROOT_DIR, 'data/bun0.pcd'));
    const cloud = pcl.io.loadPCDData<PCL.PointNormal>(data, PCL.PointNormal);
    // Get point cloud resolution
    const resolution = pcl.common.computeCloudResolution(cloud);
    const tree = new pcl.search.KdTree<PCL.PointNormal>(PCL.PointNormal);
    const keypoints = new pcl.common.PointCloud<PCL.PointNormal>(PCL.PointNormal);
    const iss = new pcl.keypoints.ISSKeypoint3D<PCL.PointNormal>(PCL.PointNormal);

    iss.setSearchMethod(tree);
    iss.setSalientRadius(6 * resolution);
    iss.setNonMaxRadius(4 * resolution);
    iss.setThreshold21(0.975);
    iss.setThreshold32(0.975);
    iss.setMinNeighbors(5);
    iss.setInputCloud(cloud);
    iss.compute(keypoints);
    const keypointsIndices = iss.getKeypointsIndices();

    expect(keypoints.points.size).toBe(6);
    expect(keypointsIndices.indices.size).toBe(6);
  });
});
