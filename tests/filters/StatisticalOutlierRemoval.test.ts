import * as PCL from '../../';

describe('StatisticalOutlierRemoval', () => {
  it('should filtering a PointCloud using a StatisticalOutlierRemoval filter', () => {
    const before = [
      [0.352222, -0.151883, -0.106395],
      [-0.397406, -0.473106, 1.292602],
      [-0.731898, 0.667105, 0.441304],
      [-0.734766, 0.854581, -0.0361733],
      [-0.4607, -0.277468, -0.916762],
      [10.4607, 20.277468, 40.916762],
    ];

    const cloud = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);
    const cloudFiltered = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

    for (let i = 0; i < before.length; i++) {
      cloud.addPoint(new PCL.PointXYZ(...before[i]));
    }

    const sor = new PCL.StatisticalOutlierRemoval();
    sor.setInputCloud(cloud);
    sor.setStddevMulThresh(1);
    sor.filter(cloudFiltered);

    expect(cloudFiltered.points.size).not.toBe(cloud.points.size);
  });
});
