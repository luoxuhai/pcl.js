/**
 * https://github.com/emscripten-core/emscripten/blob/main/src/library_fs.js
 * https://nodejs.org/api/fs.html#file-system
 */

export const writeFile = (
  path: string,
  data: string | ArrayBufferView,
  opts?: { flags?: string | undefined },
) => __PCLCore__.FS.writeFile(path, data, opts);

export function readFile(
  path: string,
  opts?: { encoding: 'binary' | 'utf8'; flags?: string },
): string | Uint8Array {
  return __PCLCore__.FS.readFile(path, opts);
}
export const unlink = (path: string) => __PCLCore__.FS.unlink(path);
export const rename = (oldPath: string, newPath: string) => __PCLCore__.FS.rename(oldPath, newPath);
export const mkdir = (path: string, mode?: number) => __PCLCore__.FS.mkdir(path, mode);
export const rmdir = (path: string) => __PCLCore__.FS.rmdir(path);
export const readdir = (path: string) => __PCLCore__.FS.readdir(path);

export function stat(path: string, dontFollow?: boolean) {
  const { FS } = __PCLCore__;
  const info = FS.stat(path, dontFollow);
  return {
    ...info,
    isDir: FS.isDir(info.mode),
    isFile: FS.isFile(info.mode),
  };
}
