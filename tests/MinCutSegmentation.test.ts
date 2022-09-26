import fs from 'fs';
import path from 'path';
import * as PCL from '../';

describe('MinCutSegmentation', () => {
  it('should segment a PointCloud into foreground and background', () => {
    const pcl = (window as any).pcl as PCL.PCLInstance;

    const filename = 'min_cut_segmentation_tutorial.pcd';
    const pcd = fs.readFileSync(path.join(__dirname, `../data/${filename}`));
    pcl.fs.writeFile(filename, new Uint8Array(pcd));
    const cloud = pcl.io.loadPCDFile<PCL.PointXYZ>(filename, PCL.PointXYZ);

    const mcSeg = new pcl.segmentation.MinCutSegmentation<PCL.PointXYZ>(
      PCL.PointXYZ,
    );
    const objectCenter: PCL.PointXYZ = new PCL.PointXYZ(68.97, -18.55, 0.57);

    const radius = 3.0433856;
    const sigma = 0.25;
    const sourceWeight = 0.8;
    const neighborNumber = 14;

    const foregroundPoints = new pcl.common.PointCloud<PCL.PointXYZ>();
    foregroundPoints.points.push(objectCenter);

    mcSeg.setForegroundPoints(foregroundPoints);
    // mcSeg.setInputCloud(cloud);
    // mcSeg.setRadius(radius);
    // mcSeg.setSigma(sigma);
    // mcSeg.setSourceWeight(sourceWeight);
    // mcSeg.setNumberOfNeighbours(neighborNumber);

    // let clusters = new pcl.Module.vectorPointIndices();
    // mcSeg.extract(clusters);

    // print(clusters)
    // expect(clusters.size(), 2);
  });
});
