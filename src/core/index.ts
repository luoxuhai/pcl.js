import initPCLCore from '../bind/build/pcl-core';
import io from '../modules/io';
import filters from '../modules/filters';

interface InitOptions {
  arrayBuffer?: ArrayBuffer;
  url?: string;
  /**
   * @default { credentials: 'same-origin' }
   */
  fetchOptions?: RequestInit;
  onsuccess?: (module: any) => void;
  onerror?: (error: unknown) => void;
  // TODO
  // onprogress?: () => void;
}

async function init(options?: InitOptions) {
  const { arrayBuffer, url, fetchOptions } = options ?? {};

  const moduleOptions: EmscriptenWasm.ModuleOpts = {
    locateFile: (path, scriptDirectory) =>
      arrayBuffer || url || scriptDirectory + path,
    fetchSettings: fetchOptions,
  };

  let Module: EmscriptenWasm.Module;
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
    Module,
    info,
    io,
    filters,
  };
}

const PCL = {
  init,
};

export default PCL;
