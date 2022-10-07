import initPCLCore from '@/bind/build/pcl-core';
import { Emscripten } from '@/types';
import { InitOptions } from './base';

let isInitialized = false;

async function init(options?: InitOptions) {
  const {
    arrayBuffer,
    url,
    fetchOptions: fetchSettings = { credentials: 'same-origin' },
  } = options ?? {};

  const moduleOptions: Emscripten.ModuleOpts = {
    wasmBinary: arrayBuffer,
    locateFile: (path, scriptDirectory) => url || scriptDirectory + path,
    fetchSettings,
  };

  try {
    window.__PCLCore__ = await initPCLCore(moduleOptions);
    isInitialized = true;
    options?.onsuccess?.();
  } catch (error) {
    options?.onerror?.(error);
    throw error;
  }
}

function destroy() {
  window.__PCLCore__ = null;
  isInitialized = false;
}

export { init, destroy, isInitialized, InitOptions };
export * from './base';
