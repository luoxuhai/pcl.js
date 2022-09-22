import fs from 'fs';
import path from 'path';
import * as PCL from '../';

describe('RandomSample', () => {
  it('should downsampling a PointCloud using a RandomSample filter', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const filename = 'table_scene_lms400.pcd';
    const pcd = fs.readFileSync(path.join(__dirname, `../data/${filename}`));
    pcl.fs.writeFile(filename, new Uint8Array(pcd));
    const cloud = pcl.io.loadPCDFile<PCL.PointXYZ>(filename, PCL.PointXYZ);
    const sample = 1000;
    const rs = new pcl.filters.RandomSample<PCL.PointXYZ>(PCL.PointXYZ);
    rs.setInputCloud(cloud);
    rs.setSample(sample);
    rs.setSeed(1);
    const cloudFiltered = rs.filter();

    expect(cloudFiltered.points.size()).toBe(sample);
  });
});
