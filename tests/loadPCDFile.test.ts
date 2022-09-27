import * as PCL from '../';
import { writeFile } from './common';

describe('loadPCDFile', () => {
  it('should load a ascii format PCD file with XYZ fields', async () => {
    const pcl = global.pcl as PCL.PCLInstance;

    writeFile('ism_test_cat.pcd', pcl);
    const cloud = pcl.io.loadPCDFile('ism_test_cat.pcd');
    expect(cloud?.points.size).toBe(3400);
  });

  it('should load a binary_compressed format PCD file with XYZ fields', async () => {
    const pcl = global.pcl as PCL.PCLInstance;

    writeFile('room_scan1.pcd', pcl);
    const cloud = pcl.io.loadPCDFile('room_scan1.pcd');
    expect(cloud?.points.size).toBe(112586);
  });
});
