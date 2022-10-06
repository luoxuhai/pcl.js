declare const __PCLCore__: Emscripten.Module;

interface Window {
  __PCLCore__: Emscripten.Module;
}

type UnionToIntersection<U> = (U extends any ? (k: U) => void : never) extends (k: infer I) => void
  ? I
  : never;
