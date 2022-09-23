import { pointTypeMap, TPointTypesUnion } from '../modules/point-types/type';

export const ENVIRONMENT_IS_NODE =
  typeof process === 'object' &&
  typeof process.versions === 'object' &&
  typeof process.versions.node === 'string';

export function getRandomArbitrary(min: number, max: number) {
  return `${Math.ceil(Math.random() * (max - min) + min)}-${Date.now()}`;
}

export function getPointType(native: Emscripten.NativeAPI, className: string) {
  const name = native.$$.ptrType.registeredClass.name.replace(className, '');
  return pointTypeMap[name] as TPointTypesUnion;
}
