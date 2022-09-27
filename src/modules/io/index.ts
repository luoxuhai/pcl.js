import {
  PointCloud,
  PointTypesUnion,
  TPointTypesUnion,
  PointTypesIntersection,
  PointXYZ,
} from '../point-types';
import fs, { FileSystem } from '../fs';
import { getRandomArbitrary } from '../../utils';

let FS: FileSystem;

function loadPCDFile<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
>(filename: string, _PT: TPointTypesUnion = PointXYZ) {
  const cloud = new PointCloud<T>(_PT);
  const status = __PCLCore__[`loadPCDFile${_PT.name}`](filename, cloud.native);
  const isSuccess = status === 0;
  if (!isSuccess) {
    cloud.delete();
    throw Error("Couldn't load the pcd data");
  }
  return cloud;
}

function savePCDFile(filename: string, cloud: PointCloud, binaryMode = false) {
  const flag = __PCLCore__[`savePCDFile${cloud._PT.name}`](
    filename,
    cloud.native,
    binaryMode,
  );
  return flag === 0;
}

function savePCDFileASCII(filename: string, cloud: PointCloud) {
  return savePCDFile(filename, cloud);
}

function savePCDFileBinary(filename: string, cloud: PointCloud) {
  return savePCDFile(filename, cloud, true);
}

function savePCDFileBinaryCompressed(filename: string, cloud: PointCloud) {
  const flag = __PCLCore__[`savePCDFileBinaryCompressed${cloud._PT.name}`](
    filename,
    cloud.native,
  );
  return flag === 0;
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

function loadPCDData<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
>(data: ArrayBuffer, _PT: TPointTypesUnion = PointXYZ) {
  if (!FS) {
    FS = fs();
  }

  const filename = `temp-${getRandomArbitrary(0, 10000)}.pcd`;
  FS.writeFile(filename, new Uint8Array(data));
  const cloud = loadPCDFile<T>(filename, _PT);
  FS.unlink(filename);

  return cloud;
}

function savePCDData(
  cloud: PointCloud<Partial<PointTypesIntersection>>,
  binaryMode = false,
) {
  if (!FS) {
    FS = fs();
  }

  const filename = `temp-${getRandomArbitrary(0, 10000)}.pcd`;
  savePCDFile(filename, cloud, binaryMode);
  const data = FS.readFile(filename);
  FS.unlink(filename);

  return data;
}

function savePCDDataASCII(cloud: PointCloud<Partial<PointTypesIntersection>>) {
  return savePCDData(cloud);
}

function savePCDDataBinary(cloud: PointCloud<Partial<PointTypesIntersection>>) {
  return savePCDData(cloud, true);
}

function savePCDDataBinaryCompressed(
  cloud: PointCloud<Partial<PointTypesIntersection>>,
) {
  if (!FS) {
    FS = fs();
  }

  const filename = `temp-${getRandomArbitrary(0, 10000)}.pcd`;
  savePCDFileBinaryCompressed(filename, cloud);
  const data = FS.readFile(filename);
  FS.unlink(filename);

  return data;
}

export default {
  loadPCDFile,
  savePCDFile,
  savePCDFileASCII,
  savePCDFileBinary,
  savePCDFileBinaryCompressed,
  readPCDHeader,
  loadPCDData,
  savePCDData,
  savePCDDataASCII,
  savePCDDataBinary,
  savePCDDataBinaryCompressed,
};
