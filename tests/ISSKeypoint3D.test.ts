import fs from 'fs';
import path from 'path';
import * as PCL from '../';

describe('ISSKeypoint3D', () => {
  it('should compute the ISS 3D keypoints - Without Boundary Estimation', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const data = fs.readFileSync(path.join(__dirname, '../data/bun0.pcd'));
    const cloud = pcl.io.loadPCDData<PCL.PointXYZ>(data, PCL.PointXYZ);
    // Get point cloud resolution
    const resolution = pcl.common.computeCloudResolution(cloud);
    const tree = new pcl.search.KdTree<PCL.PointXYZ>(PCL.PointXYZ);
    const keypoints = new pcl.common.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);
    const iss = new pcl.keypoints.ISSKeypoint3D<PCL.PointXYZ>(PCL.PointXYZ);

    iss.setSearchMethod(tree);
    iss.setSalientRadius(6 * resolution);
    iss.setNonMaxRadius(4 * resolution);
    iss.setThreshold21(0.975);
    iss.setThreshold32(0.975);
    iss.setMinNeighbors(5);
    iss.setInputCloud(cloud);
    iss.compute(keypoints);

    expect(keypoints.points.size).toBe(6);
  });
});
