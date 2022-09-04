import * as PCL from '../';

const url = 'http://localhost:4321/pcl-core.wasm';

export function initPCL() {
  return PCL.init({
    url,
    log: false,
  });
}
