/* eslint-disable @typescript-eslint/ban-ts-comment */
// @ts-nocheck
import fs from 'fs';
import path from 'path';
import * as PCL from '../';

export function initPCL() {
  const wasm = fs.readFileSync(path.join(global.ROOT_DIR, 'dist/pcl-core.wasm'));
  return PCL.init({
    arrayBuffer: wasm,
  });
}

export function writeFile(name: string, FS: PCL.fs) {
  const pcd = fs.readFileSync(path.join(global.ROOT_DIR, `data/${name}`));
  FS.writeFile(name, new Uint8Array(pcd));
}

export function getTestPCDFile(filename: string) {
  return fs.readFileSync(path.join(global.ROOT_DIR, `data/${filename}`));
}
