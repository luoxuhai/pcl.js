import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('InitializePCL', () => {
  it('should initialize pcl.js via ArrayBuffer', async () => {
    const wasm = fs.readFileSync(path.join(global.ROOT_DIR, 'dist/pcl-core.wasm'));
    await PCL.init({
      arrayBuffer: wasm,
    });

    expect(PCL.isInitialized).toBe(true);
  });

  it('should destroy a pcl.js instance', async () => {
    expect(PCL.isInitialized).toBe(true);
    PCL.destroy();
    expect(PCL.isInitialized).toBe(false);
  });
});
