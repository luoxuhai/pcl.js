interface PointCloud {
  width: number;
  height: number;
}

declare const __PCLCore__: EmscriptenWasm.Module;

interface Window {
  __PCLCore__: Module;
}
