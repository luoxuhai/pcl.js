// https://github.com/GoogleChromeLabs/squoosh/blob/dev/emscripten-types.d.ts

declare namespace Emscripten {
  type EnvironmentType = 'WEB' | 'NODE' | 'SHELL' | 'WORKER';

  interface ModuleOpts {
    wasmBinary?: ArrayBuffer | Uint8Array | Int8Array;
    mainScriptUrlOrBlob?: string;
    noInitialRun?: boolean;
    fetchSettings?: RequestInit;
    locateFile?: (path: string, scriptDirectory: string) => string;
    onRuntimeInitialized?: () => void;
  }

  interface FS {
    //
    // nodes
    //
    isFile(mode: number): boolean;
    isDir(mode: number): boolean;
    isLink(mode: number): boolean;
    isChrdev(mode: number): boolean;
    isBlkdev(mode: number): boolean;
    isFIFO(mode: number): boolean;
    isSocket(mode: number): boolean;

    //
    // core
    //
    mkdir(path: string, mode?: number): void;
    rmdir(path: string): void;
    readdir(path: string): any;
    rename(oldPath: string, newPath: string): any;
    unlink(path: string): any;
    stat(
      path: string,
      dontFollow?: boolean,
    ): {
      blksize: number;
      blocks: number;
      dev: number;
      gid: number;
      ino: number;
      mode: 33206;
      nlink: number;
      rdev: number;
      size: number;
      uid: number;
      atime: Date;
      mtime: Date;
      ctime: Date;
      isDir: boolean;
      isFile: boolean;
    };
    readFile(
      path: string,
      opts?: { encoding?: 'binary'; flags?: string | undefined },
    ): Uint8Array;
    readFile(
      path: string,
      opts?: { encoding: 'utf8'; flags?: string | undefined },
    ): string;
    writeFile(
      path: string,
      data: string | ArrayBufferView,
      opts?: { flags?: string | undefined },
    ): any;
    cwd(): string;
  }

  interface Module {
    [key: string]: any;
    print(str: string): void;
    printErr(str: string): void;
    arguments: string[];
    environment: EnvironmentType;
    preInit: { (): void }[];
    preRun: { (): void }[];
    postRun: { (): void }[];
    preinitializedWebGLContext: WebGLRenderingContext;
    noInitialRun: boolean;
    noExitRuntime: boolean;
    logReadFiles: boolean;
    filePackagePrefixURL: string;
    wasmBinary: ArrayBuffer;

    destroy(object: object): void;
    getPreloadedPackage(
      remotePackageName: string,
      remotePackageSize: number,
    ): ArrayBuffer;
    instantiateWasm(
      imports: WebAssembly.Imports,
      successCallback: (module: WebAssembly.Module) => void,
    ): WebAssembly.Exports;
    locateFile(url: string): string;
    onCustomMessage(event: MessageEvent): void;

    Runtime: any;

    ccall(
      ident: string,
      returnType: string | null,
      argTypes: string[],
      args: any[],
    ): any;
    cwrap(ident: string, returnType: string | null, argTypes: string[]): any;

    setValue(ptr: number, value: any, type: string, noSafe?: boolean): void;
    getValue(ptr: number, type: string, noSafe?: boolean): number;

    ALLOC_NORMAL: number;
    ALLOC_STACK: number;
    ALLOC_STATIC: number;
    ALLOC_DYNAMIC: number;
    ALLOC_NONE: number;

    allocate(slab: any, types: string, allocator: number, ptr: number): number;
    allocate(
      slab: any,
      types: string[],
      allocator: number,
      ptr: number,
    ): number;

    Pointer_stringify(ptr: number, length?: number): string;
    UTF16ToString(ptr: number): string;
    stringToUTF16(str: string, outPtr: number): void;
    UTF32ToString(ptr: number): string;
    stringToUTF32(str: string, outPtr: number): void;

    // USE_TYPED_ARRAYS == 1
    HEAP: Int32Array;
    IHEAP: Int32Array;
    FHEAP: Float64Array;

    // USE_TYPED_ARRAYS == 2
    HEAP8: Int8Array;
    HEAP16: Int16Array;
    HEAP32: Int32Array;
    HEAPU8: Uint8Array;
    HEAPU16: Uint16Array;
    HEAPU32: Uint32Array;
    HEAPF32: Float32Array;
    HEAPF64: Float64Array;

    TOTAL_STACK: number;
    TOTAL_MEMORY: number;
    FAST_MEMORY: number;

    addOnPreRun(cb: () => any): void;
    addOnInit(cb: () => any): void;
    addOnPreMain(cb: () => any): void;
    addOnExit(cb: () => any): void;
    addOnPostRun(cb: () => any): void;

    // Tools
    intArrayFromString(
      stringy: string,
      dontAddNull?: boolean,
      length?: number,
    ): number[];
    intArrayToString(array: number[]): string;
    writeStringToMemory(
      str: string,
      buffer: number,
      dontAddNull: boolean,
    ): void;
    writeArrayToMemory(array: number[], buffer: number): void;
    writeAsciiToMemory(str: string, buffer: number, dontAddNull: boolean): void;

    addRunDependency(id: any): void;
    removeRunDependency(id: any): void;

    preloadedImages: any;
    preloadedAudios: any;

    _malloc(size: number): number;
    _free(ptr: number): void;

    onRuntimeInitialized: () => void | null;

    FS: FS;
  }

  interface PtrType {
    isConst: boolean;
    isReference: boolean;
    isSmartPointer: boolean;
    name: string;
    registeredClass: {
      name: string;
    };
  }

  interface NativeAPI {
    [key: string]: any;
    $$: {
      count: { value: number };
      ptr: number;
      ptrType: PtrType;
      smartPtr?: number;
      smartPtrType?: PtrType;
    };
    clone: () => void;
    delete: () => void;
    deleteLater: () => void;
    isDeleted: () => boolean;
  }

  type ModuleFactory<T extends Module = Module> = (
    moduleOverrides?: ModuleOpts,
  ) => Promise<T>;
}
