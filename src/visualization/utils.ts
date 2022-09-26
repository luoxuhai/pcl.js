import { BufferGeometry, Vector3 } from 'three';

export function getCenter(geometry: BufferGeometry) {
  geometry.computeBoundingBox();
  const center = new Vector3();
  geometry.boundingBox?.getCenter(center);
  return center;
}
