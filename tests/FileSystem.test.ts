import fs from 'fs';
import path from 'path';
import * as PCL from '../';
import { initPCL } from './common';

let pcl: PCL.PCLInstance;
beforeAll(async () => {
  pcl = (await initPCL())!;
});

describe('FileSystem', () => {
  const text = '# .PCD v.7 - Point Cloud Data file format';

  it('should write and read files', async () => {
    const binary = fs.readFileSync(
      path.join(__dirname, '../data/room_scan2.pcd'),
    );
    pcl?.fs.writeFile('test-write-file.pcd', text);
    pcl?.fs.writeFile('test-write-binary-file.pcd', new Uint8Array(binary));
    const textRes = pcl?.fs.readFile('test-write-file.pcd', {
      encoding: 'utf8',
    });
    const binaryRes = pcl?.fs.readFile('test-write-binary-file.pcd');

    expect(textRes).toBe(text);
    expect(binaryRes?.byteLength).toBe(binary.byteLength);
  });

  it('should read a file information', async () => {
    pcl?.fs.writeFile('test-read-file-info.pcd', text);
    const pcd = pcl?.fs.stat('test-read-file-info.pcd');

    expect(pcd?.size).toBe(41);
  });

  it('should create a folder', async () => {
    pcl?.fs.mkdir('new-folder');
    const result = pcl?.fs.stat('new-folder');

    expect(result?.size).toBe(4096);
    expect(result?.isDir).toBe(true);
  });
});
