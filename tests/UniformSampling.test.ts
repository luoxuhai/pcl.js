import fs from 'fs';
import path from 'path';
import * as PCL from '../';

describe('UniformSampling', () => {
  it('should downsampling a PointCloud using a UniformSampling filter', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const filename = 'table_scene_lms400.pcd';
    const pcd = fs.readFileSync(path.join(__dirname, `../data/${filename}`));
    pcl.fs.writeFile(filename, new Uint8Array(pcd));
    const cloud = pcl.io.loadPCDFile<PCL.PointXYZI>(filename, PCL.PointXYZI);
    const cloudFiltered = new pcl.common.PointCloud<PCL.PointXYZI>(
      PCL.PointXYZI,
    );

    const us = new pcl.filters.UniformSampling<PCL.PointXYZI>(PCL.PointXYZI);
    us.setInputCloud(cloud);
    us.setRadiusSearch(0.005);
    us.filter(cloudFiltered);

    expect(cloudFiltered.points.size()).toBe(141525);
  });
});
