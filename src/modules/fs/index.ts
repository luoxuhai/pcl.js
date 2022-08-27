/**
 * https://github.com/emscripten-core/emscripten/blob/main/src/library_fs.js
 * https://nodejs.org/api/fs.html#file-system
 */

export default () => {
  const writeFile = __PCLCore__.FS.writeFile;
  const readFile = __PCLCore__.FS.readFile;
  const unlink = __PCLCore__.FS.unlink;
  const rename = __PCLCore__.FS.rename;
  const mkdir = __PCLCore__.FS.mkdir;
  const rmdir = __PCLCore__.FS.rmdir;
  const readdir = __PCLCore__.FS.readdir;
  const stat = __PCLCore__.FS.stat;
  const isFile = __PCLCore__.FS.isFile;
  const isDir = __PCLCore__.FS.isDir;

  return {
    mkdir,
    rmdir,
    rename,
    unlink,
    stat,
    writeFile,
    readFile,
    isFile,
    isDir,
    readdir,
  };
};
