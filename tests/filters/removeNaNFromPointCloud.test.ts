import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('removeNaNFromPointCloud and removeNaNNormalsFromPointCloud', () => {
  const data = fs.readFileSync(path.join(__dirname, 'exist-nan.pcd'));

  it('should removes points with x, y, or z equal to NaN', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const cloudIn = pcl.io.loadPCDData(data);

    const { cloud, indices } = pcl.filters.removeNaNFromPointCloud(cloudIn);

    expect(cloud.size).toBe(8);
    expect(indices.get(0)).toBe(1);
  });

  it('should removes points that have their normals invalid (i.e., equal to NaN)', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const cloudIn = pcl.io.loadPCDData<PCL.PointNormal>(data, PCL.PointNormal);

    const { cloud, indices } =
      pcl.filters.removeNaNNormalsFromPointCloud(cloudIn);

    expect(cloud.size).toBe(6);
    expect(indices.get(2)).toBe(3);
  });
});
