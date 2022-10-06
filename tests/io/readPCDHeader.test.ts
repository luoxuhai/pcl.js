import fs from 'fs';
import path from 'path';
import * as PCL from '../../';
import { writeFile } from '../utils';

describe('readPCDHeader', () => {
  it('should read the header of a PCD file with an x,y,z field', async () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const filename = 'ism_test_cat.pcd';

    writeFile(filename, pcl);
    const header = pcl.io.readPCDFileHeader(filename);
    expect(header?.fields).toEqual(['x', 'y', 'z']);
  });

  it('should read the header of a PCD file with an x,y,x,intensity,distance,sid field', async () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const filename = 'table_scene_lms400.pcd';
    writeFile(filename, pcl);
    const header = pcl.io.readPCDFileHeader(filename);
    expect(header?.fields).toEqual(['x', 'y', 'z', 'intensity', 'distance', 'sid']);
  });

  it('should read the header of a PCD file using readPCDHeader', async () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const data = fs.readFileSync(path.join(global.ROOT_DIR, `data/ism_test_cat.pcd`));
    const header = pcl.io.readPCDHeader(data);
    expect(header?.fields).toEqual(['x', 'y', 'z']);
  });
});
