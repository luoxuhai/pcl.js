import {
  PointCloud,
  createPointCloud,
  PointTypesUnion,
  PointTypesMerge,
  PointTypes,
} from '../point-types';

const pointTypeMap: {
  [key: string]: PointTypes;
} = {
  xyz: PointTypes.PointXYZ,
  xyzintensity: PointTypes.PointXYZI,
  xyzrgb: PointTypes.PointXYZRGB,
  xyzrgba: PointTypes.PointXYZRGBA,
};

function loadPCDFile<
  T extends Partial<PointTypesUnion> = Partial<PointTypesMerge>,
>(filename: string, pointType = PointTypes.PointXYZ) {
  const PCLCore = __PCLCore__;

  if (!pointType) {
    let fieldNames = '';
    const fields = PCLCore.readPCDHeader(filename);
    for (let i = 0; i < fields.size(); i++) {
      fieldNames += fields.get(i).name;
    }
    fields.delete();

    pointType = pointTypeMap[fieldNames];
  }

  if (!pointType) {
    console.error('pointType cannot be empty');
    return;
  }

  const native = PCLCore[`loadPCDFile${pointType}`](filename);
  return createPointCloud<T>(native);
}

function savePCDFile(filename: string, cloud: PointCloud, binaryMode = false) {
  return __PCLCore__[`savePCDFile${cloud.type}`](
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
  return __PCLCore__[`savePCDFileBinaryCompressed${cloud.type}`](
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
