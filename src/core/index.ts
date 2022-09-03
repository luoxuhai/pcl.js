import initPCLCore from '../bind/build/pcl-core';
import fs from '../modules/fs';
import io from '../modules/io';
import filters from '../modules/filters';
import registration from '../modules/registration';
import common from '../modules/common';

interface InitOptions {
  /**
   * ArrayBuffer for pcl-core.wasm file.
   */
  arrayBuffer?: ArrayBuffer;
  /**
   * URL for pcl-core.wasm file.
   */
  url?: string;
  /**
   * Override the default settings object used when fetching the Wasm module from the network.
   *
   * @default { credentials: 'same-origin' }
   */
  fetchOptions?: RequestInit;
  onsuccess?: (module: Emscripten.Module) => void;
  onerror?: (error: unknown) => void;
  // TODO
  // onprogress?: () => void;
}

async function init(options?: InitOptions) {
  const { arrayBuffer, url, fetchOptions } = options ?? {};

  const moduleOptions: Emscripten.ModuleOpts = {
    wasmBinary: arrayBuffer,
    locateFile: (path, scriptDirectory) => url || scriptDirectory + path,
    fetchSettings: fetchOptions,
  };

  let Module: Emscripten.Module;
  try {
    Module = await initPCLCore(moduleOptions);
    window.__PCLCore__ = Module;
    options?.onsuccess?.(Module);
  } catch (error) {
    options?.onerror?.(error);
    return;
  }

  const PCL_VERSION: string = Module.PCL_VERSION;
  console.log('pcl.js version: __version__');
  console.log(`PCL version: ${PCL_VERSION}`);

  const info = {
    PCL_VERSION,
  };

  return {
    /**
     * Emscripten Module
     * {@link https://emscripten.org/docs/api_reference/module.html}
     */
    Module,
    /**
     * Base info
     */
    info,
    /**
     * File system
     * {@link https://emscripten.org/docs/api_reference/Filesystem-API.html}
     */
    fs: fs(),
    io,
    filters,
    registration,
    common,
  };
}

const VERSION = '__version__';

export { init, VERSION };
export * from '../modules/point-types/type';
