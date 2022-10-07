import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('FileSystem', () => {
  const text = '# .PCD v.7 - Point Cloud Data file format';

  it('should write and read files', () => {
    const binary = fs.readFileSync(path.join(global.ROOT_DIR, '/data/room_scan2.pcd'));
    PCL.fs.writeFile('test-write-file.pcd', text);
    PCL.fs.writeFile('test-write-binary-file.pcd', new Uint8Array(binary));
    const textRes = PCL.fs.readFile('test-write-file.pcd', {
      encoding: 'utf8',
    });
    const binaryRes = PCL.fs.readFile('test-write-binary-file.pcd') as Uint8Array;

    expect(textRes).toBe(text);
    expect(binaryRes?.byteLength).toBe(binary.byteLength);
  });

  it('should read a file information', () => {
    PCL.fs.writeFile('test-read-file-info.pcd', text);
    const pcd = PCL.fs.stat('test-read-file-info.pcd');

    expect(pcd?.size).toBe(41);
  });

  it('should create a folder', () => {
    PCL.fs.mkdir('new-folder');
    const result = PCL.fs.stat('new-folder');

    expect(result?.size).toBe(4096);
    expect(result?.isDir).toBe(true);
  });

  it('should delete a file', () => {
    const filename = 'test-write-file-1.pcd';
    PCL.fs.writeFile(filename, 'text');
    PCL.fs.unlink(filename);
    let result: any = null;
    try {
      result = PCL.fs.stat(filename);
    } catch {
      result = undefined;
    }
    expect(result).toBeUndefined();
  });
});
