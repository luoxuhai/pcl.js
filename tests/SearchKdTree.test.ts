import { writeFile } from './common';
import * as PCL from '..';

describe('SearchKdTree', () => {
  it('should find the K nearest neighbors of a specific point', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const filename = 'table_scene_lms400.pcd';
    writeFile(filename, pcl);
    const cloud = pcl.io.loadPCDFile<PCL.PointXYZ>(filename, PCL.PointXYZ);
    const point = new PCL.PointXYZ();
    const k = 10;

    const kdtree = new pcl.search.KdTree(PCL.PointXYZ);
    kdtree.setInputCloud(cloud);
    const result = kdtree.nearestKSearch(point, k);

    expect(result.indices.size).toBe(k);
    expect(result.distances.size).toBe(k);
  });

  it('should find all neighbors within some radius specified', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const filename = 'room_scan1.pcd';
    writeFile(filename, pcl);
    const cloud = pcl.io.loadPCDFile<PCL.PointXYZ>(filename, PCL.PointXYZ);
    const point = new PCL.PointXYZ();

    const kdtree = new pcl.search.KdTree(PCL.PointXYZ);
    kdtree.setInputCloud(cloud);
    const result = kdtree.radiusSearch(point, 1);

    expect(result.indices.size).toBe(24424);
    expect(result.distances.size).toBe(24424);
  });
});
