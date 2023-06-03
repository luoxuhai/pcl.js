import * as PCL from '../../';
import { getTestPCDFile } from '../utils';

describe('IterativeClosestPoint', () => {
  it('should to return a registered point cloud', () => {
    const bun0 = getTestPCDFile('bun0.pcd');
    const bun4 = getTestPCDFile('bun4.pcd');

    const source = PCL.loadPCDData<PCL.PointXYZ>(bun0, PCL.PointXYZ);
    const target = PCL.loadPCDData<PCL.PointXYZ>(bun4, PCL.PointXYZ);
    const final = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

    const icp = new PCL.IterativeClosestPoint<PCL.PointXYZ>();
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.setMaxCorrespondenceDistance(0.05);
    icp.align(final);
    let transformation = Array.from(icp.getFinalTransformation());
    expect(transformation).toBe([0.8805896043777466,-0.023652318865060806,0.47329220175743103,0,0.0364861898124218,0.999174177646637,-0.017949936911463737,0,-0.4724760055541992,0.0330747626721859,0.88072270154953,0,0.0345311164855957,-0.0015062885358929634,0.041125550866127014,1]);
    icp.align(final,[1.0,0.0,0.0,0.0,
                            0.0,1.0,0.0,0.0,
                            0.0,0.0,1.0,0.0,
                            1.0,1.0,1.0,1.0]);
    transformation = Array.from(icp.getFinalTransformation());
    expect(transformation).toBe([0.9151206016540527,-0.025158608332276344,0.5144177675247192,1,0.0710173025727272,0.997667670249939,0.023175599053502083,1,-0.4379448890686035,0.031568463891744614,0.9218482971191406,1,0.0345311164855957,-0.0015062885358929634,0.041125550866127014,1]);

    expect(final.size).toBe(source.size);
  });
});
