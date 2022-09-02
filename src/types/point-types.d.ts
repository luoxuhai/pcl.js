interface PointXYZ {
  x: number;
  y: number;
  z: number;
}

interface PointXYZI {
  x: number;
  y: number;
  z: number;
  intensity: number;
}

interface PointXYZRGB {
  x: number;
  y: number;
  rgb: number;
}

interface PointXYZRGBA {
  x: number;
  y: number;
  z: number;
  rgba: number;
}

interface Points<T> {
  get(): T;
  set(index: number, value: T): boolean;
  push_back(value: T): void;
  size(): number;
  empty(): boolean;
  clear(): boolean;
}

interface PointCloud<T = PointXYZ> {
  width: number;
  height: number;
  is_dense: boolean;
  points: Points<T>;
  isOrganized: () => boolean;
}
