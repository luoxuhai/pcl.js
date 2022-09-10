/* eslint-disable @typescript-eslint/ban-ts-comment */
// @ts-nocheck
import fs from 'fs';
import path from 'path';
import * as PCL from '../';

const wasm = fs.readFileSync(path.join(__dirname, '../dist/pcl-core.wasm'));

export function initPCL() {
  return PCL.init({
    arrayBuffer: wasm,
    log: false,
  });
}
