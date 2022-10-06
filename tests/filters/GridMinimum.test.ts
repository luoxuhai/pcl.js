import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('GridMinimum', () => {
  it('should downsampling a PointCloud using a GridMinimum filter', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const filename = 'table_scene_lms400.pcd';
    const pcd = fs.readFileSync(path.join(global.ROOT_DIR, `data/${filename}`));
    pcl.fs.writeFile(filename, new Uint8Array(pcd));
    const cloud = pcl.io.loadPCDFile<PCL.PointXYZI>(filename, PCL.PointXYZI);
    const cloudFiltered = new pcl.common.PointCloud<PCL.PointXYZI>(PCL.PointXYZI);

    const gm = new pcl.filters.GridMinimum<PCL.PointXYZI>(PCL.PointXYZI);
    gm.setInputCloud(cloud);
    gm.setResolution(0.025);
    gm.filter(cloudFiltered);

    expect(cloudFiltered.points.size).toBe(2606);
  });
});
