function loadPCDFile(filename: string): PointCloud {
  return pcl.Module['io.loadPCDFile'](filename);
}

function savePCDFile(filename: string, cloud: PointCloud, binaryMode = false) {
  return pcl.Module['io.savePCDFile'](filename, cloud, binaryMode);
}

function savePCDFileASCII(filename: string, cloud: PointCloud) {
  return savePCDFile(filename, cloud);
}

function savePCDFileBinary(filename: string, cloud: PointCloud) {
  return savePCDFile(filename, cloud, true);
}

function savePCDFileBinaryCompressed(filename: string, cloud: PointCloud) {
  return pcl.Module['io.savePCDFileBinaryCompressed'](filename, cloud);
}

export default {
  loadPCDFile,
  savePCDFile,
  savePCDFileASCII,
  savePCDFileBinary,
  savePCDFileBinaryCompressed,
};
