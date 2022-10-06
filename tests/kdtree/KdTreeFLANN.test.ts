import { writeFile } from '../utils';
import * as PCL from '../../';

describe('KdTreeFLANN', () => {
  it('should find the K nearest neighbors of a specific point', () => {
    const filename = 'table_scene_lms400.pcd';
    writeFile(filename, PCL.fs);
    const cloud = PCL.loadPCDFile<PCL.PointXYZ>(filename, PCL.PointXYZ);
    const point = new PCL.PointXYZ();
    const k = 10;

    const kdtree = new PCL.KdTreeFLANN(PCL.PointXYZ);
    kdtree.setInputCloud(cloud);
    const result = kdtree.nearestKSearch(point, k);

    expect(result.indices.size).toBe(k);
    expect(result.distances.size).toBe(k);
  });

  it('should find all neighbors within some radius specified', () => {
    const filename = 'room_scan1.pcd';
    writeFile(filename, PCL.fs);
    const cloud = PCL.loadPCDFile<PCL.PointXYZ>(filename, PCL.PointXYZ);
    const point = new PCL.PointXYZ();

    const kdtree = new PCL.KdTreeFLANN(PCL.PointXYZ);
    kdtree.setInputCloud(cloud);
    const result = kdtree.radiusSearch(point, 1);

    expect(result.indices.size).toBe(24424);
    expect(result.distances.size).toBe(24424);
  });
});
