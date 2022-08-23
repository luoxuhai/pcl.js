class VoxelGrid {
  public native: any;

  constructor() {
    this.native = new Module.VoxelGrid();
  }

  public setLeafSize(lx: number, ly: number, lz: number) {
    return this.native.setLeafSize(lx, ly, lz);
  }

  public setInputCloud(cloud: PointCloud) {
    return this.native.setInputCloud(cloud);
  }

  public getInputCloud(): PointCloud | null {
    return this.native.getInputCloud() as PointCloud | null;
  }

  public filter(output: PointCloud) {
    return this.native.filter(output);
  }
}

export default VoxelGrid;
