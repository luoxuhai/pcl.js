import { Emscripten } from '@/types';
import { ENVIRONMENT_IS_NODE } from '@/utils';

if (((ENVIRONMENT_IS_NODE ? global : window) as any).__PCLCore__) {
  console.warn('Multiple instances of pcl.js being imported.');
} else {
  ((ENVIRONMENT_IS_NODE ? global : window) as any).__PCLCore__ = null;
}

export interface InitOptions {
  /**
   * ArrayBuffer for pcl-core.wasm file.
   */
  arrayBuffer?: Emscripten.ModuleOpts['wasmBinary'];
  /**
   * URL for pcl-core.wasm file.
   */
  url?: string;
  /**
   * path for pcl-core.wasm file.
   * **Only supported by nodejs environment.**
   */
  path?: string;
  /**
   * Override the default settings object used when fetching the Wasm module from the network.
   *
   * @default { credentials: 'same-origin' }
   */
  fetchOptions?: Emscripten.ModuleOpts['fetchSettings'];
  onsuccess?: () => void;
  onerror?: (error: unknown) => void;
}

export * from '@/constants';
export * as fs from '@/modules/fs';
export * from '@/modules/common';
export * from '@/modules/io';
export * from '@/modules/sample-consensus';
export * from '@/modules/kdtree';
export * from '@/modules/search';
export * from '@/modules/filters';
export * from '@/modules/keypoints';
export * from '@/modules/segmentation';
export * from '@/modules/registration';
export * from '@/modules/features';
