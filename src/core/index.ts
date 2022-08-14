import initPCLCore from '../embind/build/pcl-core';

interface Options {
  url?: string;
  arrayBuffer?: ArrayBuffer;
  onsuccess?: () => void;
  onerror?: () => void;
  onprogress?: () => void;
}

interface Module {
  onRuntimeInitialized: () => void;
}

export async function initPCL(options?: Options) {
  const module: any = {
    locateFile(path: string, scriptDirectory: string) {
      const defaultPath = `${scriptDirectory}${path}`;

      return options?.url || defaultPath;
    },
  };

  const PCLCore = await initPCLCore(module);

  return PCLCore;
}
