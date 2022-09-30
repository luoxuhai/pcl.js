import { PointCloud, PointIndices } from '@/modules/common/PointCloud';
import {
  PointTypesUnion,
  PointTypesIntersection,
} from '@/modules/common/point-types';
import KdTree from '@/modules/kdtree/KdTree';
import PCLBase from '@/modules/common/PCLBase';

class Keypoint<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends PCLBase<T> {
  public setSearchMethod(tree: KdTree) {
    this._native.setSearchMethod(tree._native);
  }

  public getSearchMethod(): KdTree {
    return this._native.getSearchMethod();
  }

  public getSearchParameter(): number {
    return this._native.getSearchParameter();
  }

  public getKeypointsIndices() {
    return new PointIndices(this._native.getKeypointsIndices());
  }

  public setKSearch(k: number) {
    this._native.setKSearch(k);
  }

  public getKSearch(): number {
    return this._native.getKSearch();
  }

  public setRadiusSearch(radius: number) {
    this._native.setRadiusSearch(radius);
  }

  public getRadiusSearch(): number {
    return this._native.setRadiusSearch();
  }

  public compute(cloud: PointCloud<T>) {
    this._native.compute(cloud._native);
  }
}

export default Keypoint;
