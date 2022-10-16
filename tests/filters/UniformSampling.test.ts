import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('UniformSampling', () => {
  it('should downsampling a PointCloud using a UniformSampling filter', () => {
    const filename = 'table_scene_lms400.pcd';
    const pcd = fs.readFileSync(path.join(global.ROOT_DIR, `data/${filename}`));
    PCL.fs.writeFile(filename, new Uint8Array(pcd));
    const cloud = PCL.loadPCDFile<PCL.PointXYZI>(filename, PCL.PointXYZI);
    const cloudFiltered = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

    const us = new PCL.UniformSampling();
    us.setInputCloud(cloud);
    us.setRadiusSearch(0.005);
    us.filter(cloudFiltered);

    expect(cloudFiltered.points.size).toBe(141525);
  });
});
