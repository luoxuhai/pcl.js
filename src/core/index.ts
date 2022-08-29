import initPCLCore from '../bind/build/pcl-core';
import fs from '../modules/fs';
import io from '../modules/io';
import filters from '../modules/filters';
import registration from '../modules/registration';

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

  console.log('PCL initialized successfully.');

  const info = {
    version: Module.PCL_VERSION,
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
  };
}

const PCL = {
  init,
};

export default PCL;
