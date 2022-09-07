import fs from 'fs';
import path from 'path';
import * as PCL from '../';
import { initPCL } from './common';

let pcl: PCL.PCLInstance;
beforeAll(async () => {
  pcl = (await initPCL())!;
});

describe('VoxelGrid', () => {
  it('should downsampling a PointCloud using a VoxelGrid filter', () => {
    const filename = 'table_scene_lms400.pcd';
    const pcd = fs.readFileSync(path.join(__dirname, `../data/${filename}`));
    pcl.fs.writeFile(filename, new Uint8Array(pcd));
    const cloud = pcl.io.loadPCDFile<PCL.PointXYZI>(filename, PCL.PointXYZI)!;
    const cloudFiltered = new pcl.common.PointCloud<PCL.PointXYZI>(
      PCL.PointXYZI,
    );

    const vg = new pcl.filters.VoxelGrid<PCL.PointXYZI>(PCL.PointXYZI);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01, 0.01, 0.01);
    vg.filter(cloudFiltered);

    expect(cloudFiltered.points.size()).toBe(41049);
  });
});
