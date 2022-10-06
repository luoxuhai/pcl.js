import initPCLCore from '@/bind/build/pcl-core';
import { ENVIRONMENT_IS_NODE } from '@/utils';
import { Emscripten } from '@/types';

export * from '@/constants';
export * as fs from '@/modules/fs';
export * from '@/modules/io';
export * from '@/modules/common';
export * from '@/modules/kdtree';
export * from '@/modules/search';
export * from '@/modules/filters';
export * from '@/modules/keypoints';
export * from '@/modules/segmentation';
export * from '@/modules/registration';

if (window.__PCLCore__) {
  console.warn('Multiple instances of pcl.js being imported.');
} else {
  window.__PCLCore__ = null;
}

let isInitialized = false;

interface InitOptions {
  /**
   * ArrayBuffer for pcl-core.wasm file.
   */
  arrayBuffer?: Emscripten.ModuleOpts['wasmBinary'];
  /**
   * URL for pcl-core.wasm file.
   */
  url?: string;
  /**
   * Override the default settings object used when fetching the Wasm module from the network.
   *
   * @default { credentials: 'same-origin' }
   */
  fetchOptions?: Emscripten.ModuleOpts['fetchSettings'];
  onsuccess?: () => void;
  onerror?: (error: unknown) => void;
}

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
    const __PCLCore__ = await initPCLCore(moduleOptions);
    if (ENVIRONMENT_IS_NODE) {
      (global as any).__PCLCore__ = __PCLCore__;
    } else {
      window.__PCLCore__ = __PCLCore__;
    }
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

export { InitOptions, init, destroy, isInitialized };
