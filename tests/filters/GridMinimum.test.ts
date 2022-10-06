import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('GridMinimum', () => {
  it('should downsampling a PointCloud using a GridMinimum filter', () => {
    const filename = 'table_scene_lms400.pcd';
    const pcd = fs.readFileSync(path.join(global.ROOT_DIR, `data/${filename}`));
    PCL.fs.writeFile(filename, new Uint8Array(pcd));
    const cloud = PCL.loadPCDFile<PCL.PointXYZI>(filename, PCL.PointXYZI);
    const cloudFiltered = new PCL.PointCloud<PCL.PointXYZI>(PCL.PointXYZI);

    const gm = new PCL.GridMinimum<PCL.PointXYZI>(PCL.PointXYZI);
    gm.setInputCloud(cloud);
    gm.setResolution(0.025);
    gm.filter(cloudFiltered);

    expect(cloudFiltered.points.size).toBe(2606);
  });
});
