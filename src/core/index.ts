import initPCLCore from '../embind/build/pcl-core';
import io from '../modules/io';
import filters from '../modules/filters';

interface InitOptions {
  url?: string;
  arrayBuffer?: ArrayBuffer;
  onsuccess?: (module: any) => void;
  onerror?: (error: unknown) => void;
  // onprogress?: () => void;
}

async function init(options?: InitOptions) {
  const moduleOptions: any = {
    locateFile: (path: string, scriptDirectory: string) =>
      options?.url || scriptDirectory + path,
  };

  let Module: any;
  try {
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    Module = await initPCLCore(moduleOptions);
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
