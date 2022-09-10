import {
  PointCloud,
  wrapPointCloud,
  PointTypesUnion,
  TPointTypesUnion,
  PointTypesIntersection,
  PointXYZ,
} from '../point-types';

function loadPCDFile<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
>(filename: string, PT: TPointTypesUnion = PointXYZ) {
  const native = __PCLCore__[`loadPCDFile${PT.name}`](filename);
  return wrapPointCloud<T>(native);
}

function savePCDFile(filename: string, cloud: PointCloud, binaryMode = false) {
  return __PCLCore__[`savePCDFile${cloud.PT.name}`](
    filename,
    cloud.native,
    binaryMode,
  ) as PointCloud;
}

function savePCDFileASCII(filename: string, cloud: PointCloud) {
  return savePCDFile(filename, cloud);
}

function savePCDFileBinary(filename: string, cloud: PointCloud) {
  return savePCDFile(filename, cloud, true);
}

function savePCDFileBinaryCompressed(filename: string, cloud: PointCloud) {
  return __PCLCore__[`savePCDFileBinaryCompressed${cloud.PT.name}`](
    filename,
    cloud.native,
  );
}

function readPCDHeader(filename: string) {
  const header = __PCLCore__.readPCDHeader(filename) as Emscripten.NativeAPI;

  const fields: string[] = [];
  for (let i = 0; i < header.size(); i++) {
    fields.push(header.get(i).name);
  }
  header.delete();

  return {
    fields,
  };
}

export default {
  loadPCDFile,
  savePCDFile,
  savePCDFileASCII,
  savePCDFileBinary,
  savePCDFileBinaryCompressed,
  readPCDHeader,
};
