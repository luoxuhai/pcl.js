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

  cloud = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

  for (let i = 0; i < points.length; i++) {
    cloud.addPoint(new PCL.PointXYZ(...points[i]));
  }
});

describe('savePCDData', () => {
  it('should save a ascii PCD data', () => {
    const status = PCL.savePCDFileASCII('ascii.pcd', cloud);

    expect(status).toBe(true);
    expect(PCL.fs.stat('ascii.pcd').isFile).toBe(true);
  });

  it('should save a binary PCD data', () => {
    const status = PCL.savePCDFileBinary('binary.pcd', cloud);

    expect(status).toBe(true);
    expect(PCL.fs.stat('binary.pcd').isFile).toBe(true);
  });

  it('should save a binary_compressed PCD data', () => {
    const status = PCL.savePCDFileBinaryCompressed('binary_compressed.pcd', cloud);

    expect(status).toBe(true);
    expect(PCL.fs.stat('binary_compressed.pcd').isFile).toBe(true);
  });
});
