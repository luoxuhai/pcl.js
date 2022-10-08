import fs from 'fs';
import path from 'path';
import * as PCL from '../../';

describe('SACSegmentation', () => {
  it('should segment a point cloud using SACSegmentation', () => {
    const data = fs.readFileSync(path.join(global.ROOT_DIR, 'data/noisy_slice_displaced.pcd'));
    const cloud = PCL.loadPCDData<PCL.PointXYZ>(data, PCL.PointXYZ);

    const coefficients = new PCL.ModelCoefficients();
    const inliers = new PCL.PointIndices();
    const seg = new PCL.SACSegmentation<PCL.PointXYZ>();

    seg.setOptimizeCoefficients(true);
    seg.setModelType(PCL.SacModelTypes.SACMODEL_SPHERE);
    seg.setMethodType(PCL.SacMethodTypes.SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.01);
    seg.setRadiusLimits(0.03, 0.07);
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    expect(coefficients.values.get(0)).toBe(0.9987797141075134);
    expect(coefficients.values.get(1)).toBe(0.7519828081130981);
    expect(coefficients.values.get(2)).toBe(1.2455064058303833);
    expect(coefficients.values.get(3)).toBe(0.05349602922797203);
  });

  it('should segment a PointNormal type point cloud using SACSegmentation', () => {
    const data = fs.readFileSync(path.join(global.ROOT_DIR, 'data/bun0.pcd'));
    const cloud = PCL.loadPCDData<PCL.PointNormal>(data, PCL.PointNormal);

    const coefficients = new PCL.ModelCoefficients();
    const inliers = new PCL.PointIndices();
    const seg = new PCL.SACSegmentation<PCL.PointNormal>();

    seg.setOptimizeCoefficients(true);
    seg.setModelType(PCL.SacModelTypes.SACMODEL_CIRCLE3D);
    seg.setMethodType(PCL.SacMethodTypes.SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.01);
    seg.setRadiusLimits(0.03, 0.07);
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    expect(coefficients.values.get(0)).toBe(-0.006689361296594143);
    expect(coefficients.values.get(1)).toBe(0.08215627074241638);
  });
});
