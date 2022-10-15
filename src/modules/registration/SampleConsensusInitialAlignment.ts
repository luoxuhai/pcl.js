import { PointCloud, wrapPointCloud } from '@/modules/common/PointCloud';
import { XYZPointTypes, PointXYZ, FPFHSignature33 } from '@/modules/common/point-types';
import { UnionToIntersection } from '@/types/utils';
import Registration from './Registration';

class SampleConsensusInitialAlignment<
  T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>,
  FeatureT extends FPFHSignature33 = FPFHSignature33,
> extends Registration<T> {
  constructor() {
    const native = new __PCLCore__.SampleConsensusInitialAlignmentPointXYZPointXYZFPFHSignature33();
    super(native);
  }

  public setSourceFeatures(cloud: PointCloud<FeatureT>) {
    this._native.setSourceFeatures(cloud._native);
  }

  public getSourceFeatures() {
    return wrapPointCloud<FeatureT>(this._native.getSourceFeatures());
  }

  public setTargetFeatures(cloud: PointCloud<FeatureT>) {
    this._native.setTargetFeatures(cloud._native);
  }

  public getTargetFeatures() {
    return wrapPointCloud<FeatureT>(this._native.getTargetFeatures());
  }

  public setMinSampleDistance(minSampleDistance: number) {
    this._native.setMinSampleDistance(minSampleDistance);
  }

  public getMinSampleDistance(): number {
    return this._native.getMinSampleDistance();
  }

  public setNumberOfSamples(samples: number) {
    this._native.setNumberOfSamples(samples);
  }

  public getNumberOfSamples(): number {
    return this._native.getNumberOfSamples();
  }

  public setCorrespondenceRandomness(k: number) {
    this._native.setCorrespondenceRandomness(k);
  }

  public getCorrespondenceRandomness(): number {
    return this._native.getCorrespondenceRandomness();
  }
}

export default SampleConsensusInitialAlignment;
