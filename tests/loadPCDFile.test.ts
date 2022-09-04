import fs from 'fs';
import path from 'path';
import * as PCL from '../';
import { initPCL } from './common';

let pcl: PCL.PCLInstance;
beforeAll(async () => {
  pcl = (await initPCL())!;
});

function writeFile(name: string, pcl) {
  const pcd = fs.readFileSync(path.join(__dirname, `../data/${name}`));
  pcl.fs.writeFile(name, new Uint8Array(pcd));
}

describe('loadPCDFile', () => {
  it('should load a ascii format PCD file with XYZ fields', async () => {
    writeFile('ism_test_cat.pcd', pcl);
    const cloud = pcl!.io.loadPCDFile('ism_test_cat.pcd');
    expect(cloud?.points.size()).toBe(3400);
  });

  it('should load a binary_compressed format PCD file with XYZ fields', async () => {
    writeFile('room_scan1.pcd', pcl);
    const cloud = pcl!.io.loadPCDFile('room_scan1.pcd');
    expect(cloud?.points.size()).toBe(112586);
  });
});
