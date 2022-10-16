import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('VoxelGrid', () => {
  it('should downsampling a PointCloud using a VoxelGrid filter', () => {
    const filename = 'table_scene_lms400.pcd';
    const pcd = fs.readFileSync(path.join(global.ROOT_DIR, `data/${filename}`));
    PCL.fs.writeFile(filename, new Uint8Array(pcd));
    const cloud = PCL.loadPCDFile<PCL.PointXYZI>(filename, PCL.PointXYZI);
    const cloudFiltered = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

    const vg = new PCL.VoxelGrid();
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01, 0.01, 0.01);
    vg.filter(cloudFiltered);

    expect(cloudFiltered.points.size).toBe(41049);
  });
});
