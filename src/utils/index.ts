import { pointTypeMap, PointTypesTypeof } from '../modules/common/point-types';
import { Emscripten } from '@/types';
import { PointCloud, PointXYZ, toXYZPointCloud, XYZPointTypes } from '@/modules/common';

export const ENVIRONMENT_IS_NODE =
  typeof process === 'object' &&
  typeof process.versions === 'object' &&
  typeof process.versions.node === 'string';

export function getRandomArbitrary(min: number, max: number) {
  return `${Math.ceil(Math.random() * (max - min) + min)}-${Date.now()}`;
}

export function getPointType(native: Emscripten.NativeAPI, className: string) {
  const name = native.$$.ptrType.registeredClass.name.replace(className, '');
  return pointTypeMap[name] as PointTypesTypeof;
}

export function setInputXYZCloud(
  cloud: PointCloud<XYZPointTypes>,
  input: (cloud: Emscripten.NativeAPI) => void,
) {
  if (cloud._PT === PointXYZ) {
    input(cloud._native);
  } else {
    const cloudXYZ = new PointCloud<PointXYZ>();
    toXYZPointCloud(cloud, cloudXYZ);
    input(cloudXYZ._native);
    cloudXYZ.manager.delete();
  }
}
