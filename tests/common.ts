/* eslint-disable @typescript-eslint/ban-ts-comment */
// @ts-nocheck
import fs from 'fs';
import path from 'path';
import { performance } from 'perf_hooks';
import * as PCL from '../';

global.performance = performance;

const wasm = fs.readFileSync(path.join(__dirname, '../dist/pcl-core.wasm'));

export function initPCL() {
  return PCL.init({
    arrayBuffer: wasm,
    log: false,
  });
}
