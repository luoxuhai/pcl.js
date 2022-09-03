export interface Point {
  x: number;
  y: number;
  z: number;
}

export type PointXYZ = Point;

export interface PointXYZI extends Point {
  intensity: number;
}

export interface PointXYZRGB extends Point {
  rgb: number;
}

export interface PointXYZRGBA extends Point {
  rgba: number;
}

export interface Normal {
  normal_x: number;
  normal_y: number;
  normal_z: number;
  curvature: number;
}

export interface PointNormal extends Point {
  normal_x: number;
  normal_y: number;
  normal_z: number;
  curvature: number;
}

export interface Points<T> {
  get(): T;
  set(index: number, value: T): boolean;
  push_back(value: T): void;
  size(): number;
  empty(): boolean;
  clear(): boolean;
}

export type PointTypesUnion =
  | PointXYZ
  | PointXYZI
  | PointXYZRGB
  | PointXYZRGBA
  | Normal
  | PointNormal;

export type PointTypesMerge = PointXYZ &
  PointXYZI &
  PointXYZRGB &
  PointXYZRGBA &
  Normal &
  PointNormal;

export enum PointTypes {
  PointXYZ = 'PointXYZ',
  PointXYZI = 'PointXYZI',
  PointXYZRGB = 'PointXYZRGB',
  PointXYZRGBA = 'PointXYZRGBA',
  Normal = 'Normal',
  PointNormal = 'PointNormal',
}
