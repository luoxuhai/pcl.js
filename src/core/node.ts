import initPCLCore from '@/bind/build/pcl-core.node';
import { Emscripten } from '@/types';
import { InitOptions } from './base';
import https from 'http';

let isInitialized = false;

async function init(options?: InitOptions) {
  const { arrayBuffer, path, url } = options ?? {};

  let wasmBinary = arrayBuffer;
  if (url) {
    wasmBinary = await fetchWASM(url);
  }

  const moduleOptions: Emscripten.ModuleOpts = {
    wasmBinary,
    locateFile: (p, scriptDirectory) => path || scriptDirectory + p,
  };

  try {
    (global as any).__PCLCore__ = await initPCLCore(moduleOptions);
    isInitialized = true;
    options?.onsuccess?.();
  } catch (error) {
    options?.onerror?.(error);
    throw error;
  }
}

function fetchWASM(url: string): Promise<ArrayBuffer> {
  return new Promise((resolve, reject) => {
    https
      .get(url, (res) => {
        res.on('data', (data) => {
          resolve(data);
        });
      })
      .on('error', reject);
  });
}

function destroy() {
  (global as any).__PCLCore__ = null;
  isInitialized = false;
}

export { init, destroy, isInitialized, InitOptions };
export * from './base';
