import * as PCL from '../../';
import { getTestPCDFile } from '../utils';

describe('PointCloud', () => {
  it('should create a point cloud data with XYZ fields, `width` = 5, `height` = 1', async () => {
    const SIZE = 5;
    const cloud = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

    const p = new PCL.PointXYZ(1, 2, 3);
    cloud.resize(SIZE, p);

    expect(cloud.points.size).toBe(SIZE);
    expect(cloud.width).toBe(SIZE);
    expect(cloud.height).toBe(1);
    expect(cloud.points.get(0)).toEqual(p);
  });

  it('should delete a point cloud C++ object', async () => {
    const cloud = new PCL.PointCloud();

    expect(cloud.manager.isDeleted()).toBe(false);
    cloud.manager.delete();
    expect(cloud.manager.isDeleted()).toBe(true);
  });

  it('should get `size` and `data`', () => {
    const data = getTestPCDFile('bun4.pcd');
    const cloud = PCL.loadPCDData(data);

    expect(cloud.size).toBe(361);
    expect(cloud.data.length).toBe(361);
  });
});
