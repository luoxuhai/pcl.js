import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('ApproximateVoxelGrid', () => {
  it('should downsampling a PointCloud using a ApproximateVoxelGrid filter', () => {
    const filename = 'table_scene_lms400.pcd';
    const pcd = fs.readFileSync(path.join(global.ROOT_DIR, `data/${filename}`));
    PCL.fs.writeFile(filename, new Uint8Array(pcd));
    const cloud = PCL.loadPCDFile<PCL.PointXYZ>(filename, PCL.PointXYZ);

    const avg = new PCL.ApproximateVoxelGrid<PCL.PointXYZ>(PCL.PointXYZ);
    avg.setInputCloud(cloud);
    avg.setLeafSize(0.01, 0.01, 0.01);
    avg.setDownsampleAllData(false);
    const cloudFiltered = avg.filter();

    expect(cloudFiltered?.points.size).toBe(96641);
  });
});
