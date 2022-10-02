import * as PCL from '../../';

let cloud: any;

beforeAll(async () => {
  const points = [
    [0.352222, -0.151883, -0.106395],
    [-0.397406, -0.473106, 1.292602],
    [-0.731898, 0.667105, 0.441304],
    [-0.734766, 0.854581, -0.0361733],
    [-0.4607, -0.277468, -0.916762],
  ];
  const pcl = global.pcl as PCL.PCLInstance;
  cloud = new pcl.common.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

  for (let i = 0; i < points.length; i++) {
    cloud.addPoint(new PCL.PointXYZ(...points[i]));
  }
});

describe('savePCDFile', () => {
  it('should save a ascii PCD file', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const result = pcl.io.savePCDDataASCII(cloud);

    expect(result.byteLength).toBe(326);
  });

  it('should save a binary PCD file', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const result = pcl.io.savePCDDataBinary(cloud);

    expect(result.byteLength).toBe(224);
  });

  it('should save a binary_compressed PCD file', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const result = pcl.io.savePCDDataBinaryCompressed(cloud);

    expect(result.byteLength).toBe(245);
  });
});
