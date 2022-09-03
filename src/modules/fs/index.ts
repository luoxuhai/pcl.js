/**
 * https://github.com/emscripten-core/emscripten/blob/main/src/library_fs.js
 * https://nodejs.org/api/fs.html#file-system
 */

export default () => {
  const { FS } = __PCLCore__;

  const writeFile = FS.writeFile;
  const readFile = FS.readFile;
  const unlink = FS.unlink;
  const rename = FS.rename;
  const mkdir = FS.mkdir;
  const rmdir = FS.rmdir;
  const readdir = FS.readdir;
  const stat = FS.stat;

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
