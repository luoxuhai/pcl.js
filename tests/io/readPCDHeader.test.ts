import fs from 'fs';
import path from 'path';
import * as PCL from '../../';
import { writeFile } from '../utils';

describe('readPCDHeader', () => {
  it('should read the header of a PCD file with an x,y,z field', async () => {
    const filename = 'ism_test_cat.pcd';

    writeFile(filename, PCL.fs);
    const header = PCL.readPCDFileHeader(filename);
    expect(header?.fields).toEqual(['x', 'y', 'z']);
  });

  it('should read the header of a PCD file with an x,y,x,intensity,distance,sid field', async () => {
    const filename = 'table_scene_lms400.pcd';
    writeFile(filename, PCL.fs);
    const header = PCL.readPCDFileHeader(filename);
    expect(header?.fields).toEqual(['x', 'y', 'z', 'intensity', 'distance', 'sid']);
  });

  it('should read the header of a PCD file using readPCDHeader', async () => {
    const data = fs.readFileSync(path.join(global.ROOT_DIR, `data/ism_test_cat.pcd`));
    const header = PCL.readPCDHeader(data);
    expect(header?.fields).toEqual(['x', 'y', 'z']);
  });
});
