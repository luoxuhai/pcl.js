import * as PCL from '../../';
import { getTestPCDFile } from '../utils';

describe('SampleConsensusInitialAlignment', () => {
  it('should using Fast Point Feature Histograms (FPFH) for 3D Registration', async () => {
    const bun0 = getTestPCDFile('bun0.pcd');
    const bun4 = getTestPCDFile('bun4.pcd');

    const cloudSource = PCL.loadPCDData<PCL.PointXYZ>(bun0, PCL.PointXYZ);
    const cloudTarget = PCL.loadPCDData<PCL.PointXYZ>(bun4, PCL.PointXYZ);
    const cloudReg = new PCL.PointCloud<PCL.PointXYZ>(PCL.PointXYZ);

    // Initialize estimators for surface normals and FPFH features
    const tree = new PCL.SearchKdTree<PCL.PointXYZ>(PCL.PointXYZ);

    const normEst = new PCL.NormalEstimation();
    normEst.setSearchMethod(tree);
    normEst.setRadiusSearch(0.05);
    const normals = new PCL.PointCloud<PCL.Normal>(PCL.Normal);

    const fpfhEst = new PCL.FPFHEstimation();
    fpfhEst.setSearchMethod(tree);
    fpfhEst.setRadiusSearch(0.05);
    const featuresSource = new PCL.PointCloud<PCL.FPFHSignature33>(PCL.FPFHSignature33);
    const featuresTarget = new PCL.PointCloud<PCL.FPFHSignature33>(PCL.FPFHSignature33);

    // Estimate the FPFH features for the source cloud
    normEst.setInputCloud(cloudSource);
    normEst.compute(normals);
    fpfhEst.setInputCloud(cloudSource);
    fpfhEst.setInputNormals(normals);
    fpfhEst.compute(featuresSource);

    // Estimate the FPFH features for the target cloud
    normEst.setInputCloud(cloudTarget);
    normEst.compute(normals);
    fpfhEst.setInputCloud(cloudTarget);
    fpfhEst.setInputNormals(normals);
    fpfhEst.compute(featuresTarget);

    // Initialize Sample Consensus Initial Alignment (SAC-IA)
    const reg = new PCL.SampleConsensusInitialAlignment();
    reg.setMinSampleDistance(0.05);
    reg.setMaxCorrespondenceDistance(0.1);
    reg.setMaximumIterations(1000);

    reg.setInputSource(cloudSource);
    reg.setInputTarget(cloudTarget);
    reg.setSourceFeatures(featuresSource);
    reg.setTargetFeatures(featuresTarget);

    // Register
    reg.align(cloudReg);

    expect(reg.getFitnessScore()).toBeLessThan(0.0005);
    expect(cloudReg.size).toBe(cloudSource.size);
  });
});
