interface PointCloud {
  width: number;
  height: number;
}

declare const __PCLCore__: Emscripten.Module;

interface Window {
  __PCLCore__: Module;
}
