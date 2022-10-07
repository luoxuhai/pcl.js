import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('ISSKeypoint3D', () => {
  it('should compute the ISS 3D keypoints - Without Boundary Estimation', () => {
    const data = fs.readFileSync(path.join(global.ROOT_DIR, 'data/bun0.pcd'));
    const cloud = PCL.loadPCDData<PCL.PointNormal>(data, PCL.PointNormal);
    // Get point cloud resolution
    const resolution = PCL.computeCloudResolution(cloud);
    const tree = new PCL.SearchKdTree<PCL.PointNormal>(PCL.PointNormal);
    const keypoints = new PCL.PointCloud<PCL.PointNormal>(PCL.PointNormal);
    const iss = new PCL.ISSKeypoint3D<PCL.PointNormal>(PCL.PointNormal);

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
