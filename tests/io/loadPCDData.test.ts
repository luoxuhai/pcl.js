import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('loadPCDData', () => {
  it('should load a ascii format PCD data with XYZ fields', async () => {
    const data = fs.readFileSync(path.join(global.ROOT_DIR, 'data/ism_test_cat.pcd'));

    const cloud = PCL.loadPCDData(data);
    expect(cloud?.points.size).toBe(3400);
  });

  it('should load a binary_compressed format PCD data with XYZ fields', async () => {
    const data = fs.readFileSync(path.join(global.ROOT_DIR, 'data/room_scan1.pcd'));

    const cloud = PCL.loadPCDData(data);
    expect(cloud?.points.size).toBe(112586);
  });
});
