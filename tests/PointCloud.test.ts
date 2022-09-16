import * as PCL from '../';

describe('PointCloud', () => {
  it('should create a point cloud data with XYZ fields, `width` = 5, `height` = 1', async () => {
    const pcl = (window as any).pcl as PCL.PCLInstance;

    const SIZE = 5;
    const cloud = new pcl.common.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

    const p = new PCL.PointXYZ(1, 2, 3);
    cloud.resize(SIZE, p);

    expect(cloud.points.size()).toBe(SIZE);
    expect(cloud.width).toBe(SIZE);
    expect(cloud.height).toBe(1);
    expect(cloud.points.get(0)).toEqual(p);
  });
});
