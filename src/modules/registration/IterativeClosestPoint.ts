import { XYZPointTypes, PointXYZ } from '@/modules/common/point-types';
import { UnionToIntersection } from '@/types/utils';
import Registration from './Registration';

class IterativeClosestPoint<
  T extends XYZPointTypes = PointXYZ & Partial<UnionToIntersection<XYZPointTypes>>,
> extends Registration<T> {
  constructor() {
    const native = new __PCLCore__.IterativeClosestPointPointXYZPointXYZ();
    super(native);
  }

  public setUseReciprocalCorrespondences(useReciprocalCorrespondence: boolean) {
    this._native.setUseReciprocalCorrespondences(useReciprocalCorrespondence);
  }

  public getUseReciprocalCorrespondences(): boolean {
    return this._native.getUseReciprocalCorrespondences();
  }
}

export default IterativeClosestPoint;
