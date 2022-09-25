/// <reference path="../emscripten.d.ts" />
/// <reference path="../global.d.ts" />

import initPCLCore from '../bind/build/pcl-core';
import fs from '../modules/fs';
import io from '../modules/io';
import filters from '../modules/filters';
import registration from '../modules/registration';
import common from '../modules/common';
import kdtree from '../modules/kdtree';
import search from '../modules/search';
import keypoints from '../modules/keypoints';
import { ENVIRONMENT_IS_NODE } from '../utils';

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
  /**
   * Show log in console
   *
   * @default true
   */
  log?: boolean;
  onsuccess?: (module: Emscripten.Module) => void;
  onerror?: (error: unknown) => void;
}

interface PCLInstance {
  /**
   * Emscripten Module
   * {@link https://emscripten.org/docs/api_reference/module.html}
   */
  Module: Emscripten.Module;
  /**
   * File system
   * {@link https://emscripten.org/docs/api_reference/Filesystem-API.html}
   */
  fs: Pick<
    Emscripten.FS,
    | 'readdir'
    | 'readFile'
    | 'writeFile'
    | 'stat'
    | 'mkdir'
    | 'rmdir'
    | 'rename'
    | 'unlink'
  >;
  /**
   * Base info
   */
  info: {
    PCL_VERSION: string;
  };
  common: typeof common;
  io: typeof io;
  filters: typeof filters;
  registration: typeof registration;
  kdtree: typeof kdtree;
  search: typeof search;
  keypoints: typeof keypoints;
}

async function init(options?: InitOptions): Promise<PCLInstance | null> {
  const {
    arrayBuffer,
    url,
    fetchOptions: fetchSettings = { credentials: 'same-origin' },
    log = true,
  } = options ?? {};

  const moduleOptions: Emscripten.ModuleOpts = {
    wasmBinary: arrayBuffer,
    locateFile: (path, scriptDirectory) => url || scriptDirectory + path,
    fetchSettings,
  };

  let Module: Emscripten.Module;
  try {
    Module = await initPCLCore(moduleOptions);
    if (ENVIRONMENT_IS_NODE) {
      (global as any).__PCLCore__ = Module;
    } else {
      window.__PCLCore__ = Module;
    }
    options?.onsuccess?.(Module);
  } catch (error) {
    options?.onerror?.(error);
    throw error;
  }

  const PCL_VERSION: string = Module.PCL_VERSION;

  if (log) {
    console.log('pcl.js version: __version__');
    console.log(`PCL version: ${PCL_VERSION}`);
  }

  const info = {
    PCL_VERSION,
  };

  return {
    Module,
    info,
    fs: fs(),
    io,
    filters,
    registration,
    common,
    kdtree,
    search,
    keypoints,
  };
}

const VERSION = '__version__';

export { init, VERSION, PCLInstance, InitOptions };
export * from '../modules/point-types/type';
