import { Emscripten } from '@/types';

declare global {
  declare const __PCLCore__: Emscripten.Module;

  interface Window {
    __PCLCore__: Emscripten.Module | null;
  }
}
