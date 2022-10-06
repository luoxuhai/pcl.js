import * as PCL from '../../';
import { writeFile } from '../utils';

describe('loadPCDFile', () => {
  it('should load a ascii format PCD file with XYZ fields', async () => {
    writeFile('ism_test_cat.pcd', PCL.fs);
    const cloud = PCL.loadPCDFile('ism_test_cat.pcd');
    expect(cloud?.points.size).toBe(3400);
  });

  it('should load a binary_compressed format PCD file with XYZ fields', async () => {
    writeFile('room_scan1.pcd', PCL.fs);
    const cloud = PCL.loadPCDFile('room_scan1.pcd');
    expect(cloud?.points.size).toBe(112586);
  });
});
