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

describe('readPCDHeader.test', () => {
  it('should read the header of a PCD file with an x,y,z field', async () => {
    const filename = 'ism_test_cat.pcd';
    writeFile(filename, pcl);
    const header = pcl.io.readPCDHeader(filename);
    expect(header?.fields).toEqual(['x', 'y', 'z']);
  });

  it('should read the header of a PCD file with an x,y,x,intensity,distance,sid field', async () => {
    const filename = 'table_scene_lms400.pcd';
    writeFile(filename, pcl);
    const header = pcl.io.readPCDHeader(filename);
    expect(header?.fields).toEqual([
      'x',
      'y',
      'z',
      'intensity',
      'distance',
      'sid',
    ]);
  });
});
