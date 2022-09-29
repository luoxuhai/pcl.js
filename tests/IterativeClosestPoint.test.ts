import * as PCL from '../';

describe('IterativeClosestPoint', () => {
  it('should to return a registered point cloud', () => {
    const pcl = global.pcl as PCL.PCLInstance;

    const source = [
      [0.352222, -0.151883, -0.106395],
      [-0.397406, -0.473106, 0.292602],
      [-0.731898, 0.667105, 0.441304],
      [-0.734766, 0.854581, -0.0361733],
      [-0.4607, -0.277468, -0.916762],
    ];
    const target = [
      [1.05222, -0.151883, -0.106395],
      [0.302594, -0.473106, 0.292602],
      [-0.0318983, 0.667105, 0.441304],
      [-0.0347655, 0.854581, -0.0361733],
      [0.2393, -0.277468, -0.916762],
    ];

    const cloudIn = new pcl.common.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);
    const cloudOut = new pcl.common.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);
    const final = new pcl.common.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

    for (let i = 0; i < source.length; i++) {
      cloudIn.addPoint(new PCL.PointXYZ(...source[i]));
      cloudOut.addPoint(new PCL.PointXYZ(...target[i]));
    }

    const icp = new pcl.registration.IterativeClosestPoint<PCL.PointXYZ>(
      PCL.PointXYZ,
    );
    icp.setInputSource(cloudIn);
    icp.setInputTarget(cloudOut);
    icp.align(final);

    expect(icp.hasConverged()).toBe(true);
    expect(final.points.size).toBe(5);
  });
});
