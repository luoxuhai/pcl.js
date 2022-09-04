import fs from 'fs';
import path from 'path';
import * as PCL from '../';

describe('InitializePCL', () => {
  it('should initialize pcl.js via remote url', async () => {
    const pcl = await PCL.init({
      url: 'http://localhost:4321/pcl-core.wasm',
      log: false,
    });

    expect(pcl?.info.PCL_VERSION).toBe('1.12.1');
  });

  it('should initialize pcl.js via ArrayBuffer', async () => {
    const wasm = fs.readFileSync(path.join(__dirname, '../dist/pcl-core.wasm'));
    const pcl = await PCL.init({
      arrayBuffer: wasm,
      log: false,
    });

    expect(pcl?.info.PCL_VERSION).toBe('1.12.1');
  });
});
