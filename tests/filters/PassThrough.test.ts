import * as PCL from '../../';

describe('PassThrough', () => {
  it('should filtering a PointCloud using a PassThrough filter', () => {
    const before = [
      [0.352222, -0.151883, -0.106395],
      [-0.397406, -0.473106, 1.292602],
      [-0.731898, 0.667105, 0.441304],
      [-0.734766, 0.854581, -0.0361733],
      [-0.4607, -0.277468, -0.916762],
    ];
    const after = [
      [-0.397406, -0.473106, 1.292602],
      [-0.731898, 0.667105, 0.441304],
    ];
    const cloud = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);
    const cloudFiltered = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

    for (let i = 0; i < before.length; i++) {
      cloud.addPoint(new PCL.PointXYZ(...before[i]));
    }

    const pass = new PCL.PassThrough<PCL.PointXYZ>(PCL.PointXYZ);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName('z');
    pass.setFilterLimits(0, 2);
    pass.filter(cloudFiltered);

    expect(cloudFiltered.points.size).toBe(after.length);
    // for (let i = 0; i < cloudFiltered.points.size; i++) {
    //   expect(cloudFiltered.points.get(i)).toEqual(
    //     new PCL.PointXYZ(...after[i]),
    //   );
    // }
  });
});
