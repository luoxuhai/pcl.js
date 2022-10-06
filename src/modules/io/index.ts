import { PointCloud } from '@/modules/common/PointCloud';
import { PointTypes, PointTypesTypeof, PointXYZ } from '@/modules/common/point-types';
import * as fs from '@/modules/fs';
import { getRandomArbitrary } from '@/utils';
import { readPCDHeader, PCDHeader } from './pcd-reader';
import { UnionToIntersection } from '@/types/utils';

function loadPCDFile<T extends PointTypes = PointXYZ & Partial<UnionToIntersection<PointTypes>>>(
  filename: string,
  _PT: PointTypesTypeof = PointXYZ,
) {
  const cloud = new PointCloud<T>(_PT);
  const status = __PCLCore__[`loadPCDFile${_PT.name}`](filename, cloud._native);
  const isSuccess = status === 0;
  if (!isSuccess) {
    cloud.delete();
    throw Error("Couldn't load the pcd data");
  }
  return cloud;
}

function savePCDFile(filename: string, cloud: PointCloud<PointTypes>, binaryMode = false) {
  const flag = __PCLCore__[`savePCDFile${cloud._PT.name}`](filename, cloud._native, binaryMode);
  return flag === 0;
}

function savePCDFileASCII(filename: string, cloud: PointCloud<PointTypes>) {
  return savePCDFile(filename, cloud);
}

function savePCDFileBinary(filename: string, cloud: PointCloud<PointTypes>) {
  return savePCDFile(filename, cloud, true);
}

function savePCDFileBinaryCompressed(filename: string, cloud: PointCloud<PointTypes>) {
  const flag = __PCLCore__[`savePCDFileBinaryCompressed${cloud._PT.name}`](filename, cloud._native);
  return flag === 0;
}

function readPCDFileHeader(filename: string) {
  const data = fs.readFile(filename);

  return readPCDHeader(data as ArrayBuffer);
}

function loadPCDData<T extends PointTypes = PointXYZ & Partial<UnionToIntersection<PointTypes>>>(
  data: ArrayBuffer,
  _PT: PointTypesTypeof = PointXYZ,
) {
  const filename = `temp-${getRandomArbitrary(0, 10000)}.pcd`;
  fs.writeFile(filename, new Uint8Array(data));
  const cloud = loadPCDFile<T>(filename, _PT);
  fs.unlink(filename);

  return cloud;
}

function savePCDData(cloud: PointCloud<PointTypes>, binaryMode = false) {
  const filename = `temp-${getRandomArbitrary(0, 10000)}.pcd`;
  savePCDFile(filename, cloud, binaryMode);
  const data = fs.readFile(filename);
  fs.unlink(filename);

  return data;
}

function savePCDDataASCII(cloud: PointCloud<PointTypes>) {
  return savePCDData(cloud);
}

function savePCDDataBinary(cloud: PointCloud<PointTypes>) {
  return savePCDData(cloud, true);
}

function savePCDDataBinaryCompressed(cloud: PointCloud<PointTypes>) {
  const filename = `temp-${getRandomArbitrary(0, 10000)}.pcd`;
  savePCDFileBinaryCompressed(filename, cloud);
  const data = fs.readFile(filename);
  fs.unlink(filename);

  return data;
}

export {
  loadPCDFile,
  savePCDFile,
  savePCDFileASCII,
  savePCDFileBinary,
  savePCDFileBinaryCompressed,
  readPCDFileHeader,
  loadPCDData,
  savePCDData,
  savePCDDataASCII,
  savePCDDataBinary,
  savePCDDataBinaryCompressed,
  readPCDHeader,
  PCDHeader,
};
