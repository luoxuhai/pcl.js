/* eslint-disable @typescript-eslint/ban-ts-comment */
// @ts-nocheck
import fs from 'fs';
import path from 'path';
import * as PCL from '../';

export function initPCL() {
  const wasm = fs.readFileSync(path.join(global.ROOT_DIR, 'dist/pcl-core.wasm'));
  return PCL.init({
    arrayBuffer: wasm,
    log: false,
  });
}

export function writeFile(name: string, pcl: PCL.PCLInstance) {
  const pcd = fs.readFileSync(path.join(global.ROOT_DIR, `data/${name}`));
  pcl.fs.writeFile(name, new Uint8Array(pcd));
}
