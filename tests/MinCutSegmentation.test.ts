import fs from 'fs';
import path from 'path';
import { parseCommandLine } from 'typescript';
import * as PCL from '../dist/types/pcl';
import { Vector } from '../dist/types/pcl';
import { initPCL } from './common';

let pcl: PCL.PCLInstance;
beforeAll(async () => {
  pcl = (await initPCL())!;
});

describe('MinCutSegmentation', () => {
  it('should segment a PointCloud into foreground and background', () => {
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
    mcSeg.setInputCloud(cloud);
    mcSeg.setRadius(radius);
    mcSeg.setSigma(sigma);
    mcSeg.setSourceWeight(sourceWeight);
    mcSeg.setNumberOfNeighbours(neighborNumber);

    const clusters = new Vector<PCL.PointIndices>();
    mcSeg.extract(clusters);
    expect(clusters.size, 2);
  });
});
