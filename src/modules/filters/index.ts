import PassThrough from './PassThrough';
import VoxelGrid from './VoxelGrid';
import StatisticalOutlierRemoval from './StatisticalOutlierRemoval';
import RadiusOutlierRemoval from './RadiusOutlierRemoval';
import UniformSampling from './UniformSampling';
import RandomSample from './RandomSample';
import GridMinimum from './GridMinimum';
import LocalMaximum from './LocalMaximum';
import ApproximateVoxelGrid from './ApproximateVoxelGrid';
import { removeNaNFromPointCloud, removeNaNNormalsFromPointCloud } from './remove-invalid';

export default {
  PassThrough,
  VoxelGrid,
  StatisticalOutlierRemoval,
  RadiusOutlierRemoval,
  UniformSampling,
  RandomSample,
  GridMinimum,
  LocalMaximum,
  ApproximateVoxelGrid,
  removeNaNFromPointCloud,
  removeNaNNormalsFromPointCloud,
};
