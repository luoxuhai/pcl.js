import {
  PointCloud,
  wrapPointCloud,
  PointTypesUnion,
  TPointTypesUnion,
  PointTypesIntersection,
  PointXYZ,
  PointXYZI,
  PointXYZRGB,
  PointXYZRGBA,
} from '../point-types';

const pointTypeMap: {
  [key: string]: TPointTypesUnion;
} = {
  xyz: PointXYZ,
  xyzintensity: PointXYZI,
  xyzrgb: PointXYZRGB,
  xyzrgba: PointXYZRGBA,
};

function loadPCDFile<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
>(filename: string, PT: TPointTypesUnion = PointXYZ) {
  const PCLCore = __PCLCore__;

  if (!PT) {
    let fieldNames = '';
    const fields = PCLCore.readPCDHeader(filename);
    for (let i = 0; i < fields.size(); i++) {
      fieldNames += fields.get(i).name;
    }
    fields.delete();

    PT = pointTypeMap[fieldNames];
  }

  if (!PT) {
    console.error('PointType cannot be empty');
    return;
  }

  const native = PCLCore[`loadPCDFile${PT.name}`](filename);
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

export default {
  loadPCDFile,
  savePCDFile,
  savePCDFileASCII,
  savePCDFileBinary,
  savePCDFileBinaryCompressed,
};
