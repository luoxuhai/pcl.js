import FilterBase from './FilterBase';

class VoxelGrid extends FilterBase {
  constructor() {
    super(new __PCLCore__.VoxelGrid());
  }

  public setLeafSize(lx: number, ly: number, lz: number) {
    return this.native.setLeafSize(lx, ly, lz);
  }

  public filter(output: PointCloud) {
    return this.native.filter(output);
  }
}

export default VoxelGrid;
