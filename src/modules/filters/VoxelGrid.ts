import FilterBase from './FilterBase';

class VoxelGrid extends FilterBase {
  constructor() {
    super(new __PCLCore__.VoxelGrid());
  }

  public setLeafSize(lx: number, ly: number, lz: number) {
    return this.native.setLeafSize(lx, ly, lz);
  }
}

export default VoxelGrid;
