import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('LocalMaximum', () => {
  it('should downsampling a PointCloud using a LocalMaximum filter', () => {
    const filename = 'table_scene_lms400.pcd';
    const pcd = fs.readFileSync(path.join(global.ROOT_DIR, `data/${filename}`));
    PCL.fs.writeFile(filename, new Uint8Array(pcd));
    const cloud = PCL.loadPCDFile<PCL.PointXYZ>(filename, PCL.PointXYZ);

    const lm = new PCL.LocalMaximum<PCL.PointXYZ>(PCL.PointXYZ);
    lm.setInputCloud(cloud);
    lm.setRadius(0.025);
    const cloudFiltered = lm.filter();

    expect(cloudFiltered?.points.size).toBe(393541);
  });
});
