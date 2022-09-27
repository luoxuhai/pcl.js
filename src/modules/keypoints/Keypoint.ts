import { PointCloud } from '../point-types';
import {
  PointTypesUnion,
  PointTypesIntersection,
  Indices,
} from '../point-types/type';
import KdTree from '../kdtree/KdTree';
import PCLBase from '../common/PCLBase';

class Keypoint<
  T extends Partial<PointTypesUnion> = Partial<PointTypesIntersection>,
> extends PCLBase<T> {
  public setSearchMethod(tree: KdTree) {
    this.native.setSearchMethod(tree.native);
  }

  public getSearchMethod(): KdTree {
    return this.native.getSearchMethod();
  }

  public getSearchParameter(): number {
    return this.native.getSearchParameter();
  }

  public getKeypointsIndices() {
    return new Indices(this.native.getKeypointsIndices());
  }

  public setKSearch(k: number) {
    this.native.setKSearch(k);
  }

  public getKSearch(): number {
    return this.native.getKSearch();
  }

  public setRadiusSearch(radius: number) {
    this.native.setRadiusSearch(radius);
  }

  public getRadiusSearch(): number {
    return this.native.setRadiusSearch();
  }

  public compute(cloud: PointCloud<T>) {
    this.native.compute(cloud.native);
  }
}

export default Keypoint;
