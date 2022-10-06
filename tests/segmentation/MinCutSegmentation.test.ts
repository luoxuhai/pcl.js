import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('MinCutSegmentation', () => {
  it('should segment a PointCloud into foreground and background clusters', () => {
    const data = fs.readFileSync(
      path.join(global.ROOT_DIR, 'data/min_cut_segmentation_tutorial.pcd'),
    );
    const cloud = PCL.loadPCDData<PCL.PointXYZ>(data, PCL.PointXYZ);

    const mcSeg = new PCL.MinCutSegmentation<PCL.PointXYZ>(PCL.PointXYZ);
    const objectCenter: PCL.PointXYZ = new PCL.PointXYZ(68.97, -18.55, 0.57);

    const radius = 3.0433856;
    const sigma = 0.25;
    const sourceWeight = 0.8;
    const neighborNumber = 14;

    const foregroundPoints = new PCL.PointCloud<PCL.PointXYZ>();
    foregroundPoints.addPoint(objectCenter);

    mcSeg.setForegroundPoints(foregroundPoints);
    mcSeg.setInputCloud(cloud);
    mcSeg.setRadius(radius);
    mcSeg.setSigma(sigma);
    mcSeg.setSourceWeight(sourceWeight);
    mcSeg.setNumberOfNeighbours(neighborNumber);

    const clusters = mcSeg.extract();
    expect(clusters.size).toBe(2);
    expect(mcSeg.getMaxFlow()).toBe(5970.370432746628);
  });
});
