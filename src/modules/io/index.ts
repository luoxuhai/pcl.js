function loadPCDFile(filename: string): PointCloud {
  return __PCLCore__['io.loadPCDFile'](filename);
}

function savePCDFile(filename: string, cloud: PointCloud, binaryMode = false) {
  return __PCLCore__['io.savePCDFile'](filename, cloud, binaryMode);
}

function savePCDFileASCII(filename: string, cloud: PointCloud) {
  return savePCDFile(filename, cloud);
}

function savePCDFileBinary(filename: string, cloud: PointCloud) {
  return savePCDFile(filename, cloud, true);
}

function savePCDFileBinaryCompressed(filename: string, cloud: PointCloud) {
  return __PCLCore__['io.savePCDFileBinaryCompressed'](filename, cloud);
}

export default {
  loadPCDFile,
  savePCDFile,
  savePCDFileASCII,
  savePCDFileBinary,
  savePCDFileBinaryCompressed,
};
