import * as PCL from '../';
import { writeFile } from './common';

describe('readPCDHeader.test', () => {
  it('should read the header of a PCD file with an x,y,z field', async () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const filename = 'ism_test_cat.pcd';
    writeFile(filename, pcl);
    const header = pcl.io.readPCDHeader(filename);
    expect(header?.fields).toEqual(['x', 'y', 'z']);
  });

  it('should read the header of a PCD file with an x,y,x,intensity,distance,sid field', async () => {
    const pcl = global.pcl as PCL.PCLInstance;

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
