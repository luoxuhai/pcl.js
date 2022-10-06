import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('removeNaNFromPointCloud and removeNaNNormalsFromPointCloud', () => {
  const data = fs.readFileSync(path.join(__dirname, 'exist-nan.pcd'));

  it('should removes points with x, y, or z equal to NaN', () => {
    const cloudIn = PCL.loadPCDData(data);

    const { cloud, indices } = PCL.removeNaNFromPointCloud(cloudIn);

    expect(cloud.size).toBe(8);
    expect(indices.get(0)).toBe(1);
  });

  it('should removes points that have their normals invalid (i.e., equal to NaN)', () => {
    const cloudIn = PCL.loadPCDData<PCL.PointNormal>(data, PCL.PointNormal);

    const { cloud, indices } = PCL.removeNaNNormalsFromPointCloud(cloudIn);

    expect(cloud.size).toBe(6);
    expect(indices.get(2)).toBe(3);
  });
});
