import * as PCL from '../';

let cloud: any;

beforeAll(async () => {
  const points = [
    [0.352222, -0.151883, -0.106395],
    [-0.397406, -0.473106, 1.292602],
    [-0.731898, 0.667105, 0.441304],
    [-0.734766, 0.854581, -0.0361733],
    [-0.4607, -0.277468, -0.916762],
  ];
  const pcl = (window as any).pcl as PCL.PCLInstance;
  cloud = new pcl.common.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

  for (let i = 0; i < points.length; i++) {
    cloud.push(new PCL.PointXYZ(...points[i]));
  }
});

describe('savePCDFile', () => {
  it('should save a ascii fPCD file', () => {
    const pcl = (window as any).pcl as PCL.PCLInstance;

    const status = pcl.io.savePCDFileASCII('ascii.pcd', cloud);

    expect(status).toBe(true);
    expect(pcl.fs.stat('ascii.pcd').isFile).toBe(true);
  });

  it('should save a binary fPCD file', () => {
    const pcl = (window as any).pcl as PCL.PCLInstance;

    const status = pcl.io.savePCDFileBinary('binary.pcd', cloud);

    expect(status).toBe(true);
    expect(pcl.fs.stat('binary.pcd').isFile).toBe(true);
  });

  it('should save a binary_compressed fPCD file', () => {
    const pcl = (window as any).pcl as PCL.PCLInstance;

    const status = pcl.io.savePCDFileBinaryCompressed(
      'binary_compressed.pcd',
      cloud,
    );

    expect(status).toBe(true);
    expect(pcl.fs.stat('binary_compressed.pcd').isFile).toBe(true);
  });
});
