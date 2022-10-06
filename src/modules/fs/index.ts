/**
 * https://github.com/emscripten-core/emscripten/blob/main/src/library_fs.js
 * https://nodejs.org/api/fs.html#file-system
 */

export type FileSystem = Pick<
  Emscripten.FS,
  'readdir' | 'readFile' | 'writeFile' | 'stat' | 'mkdir' | 'rmdir' | 'rename' | 'unlink'
>;

export default (): FileSystem => {
  const { FS } = __PCLCore__;

  const writeFile = FS.writeFile;
  const readFile = FS.readFile;
  const unlink = FS.unlink;
  const rename = FS.rename;
  const mkdir = FS.mkdir;
  const rmdir = FS.rmdir;
  const readdir = FS.readdir;

  function stat(path: string, dontFollow?: boolean) {
    const info = FS.stat(path, dontFollow);
    return {
      ...info,
      isDir: FS.isDir(info.mode),
      isFile: FS.isFile(info.mode),
    };
  }

  return {
    mkdir,
    rmdir,
    rename,
    unlink,
    stat,
    writeFile,
    readFile,
    readdir,
  };
};
