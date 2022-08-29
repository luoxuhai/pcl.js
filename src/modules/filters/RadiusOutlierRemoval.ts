import FilterIndices from './FilterIndices';

class RadiusOutlierRemoval extends FilterIndices {
  constructor(extractRemovedIndices = false) {
    super(new __PCLCore__.RadiusOutlierRemoval(extractRemovedIndices));
  }

  public setRadiusSearch(radius: number) {
    return this.native.setRadiusSearch(radius);
  }

  public getRadiusSearch(): number {
    return this.native.getRadiusSearch();
  }

  public setMinNeighborsInRadius(minPts: number) {
    return this.native.setMinNeighborsInRadius(minPts);
  }

  public getMinNeighborsInRadius(): number {
    return this.native.getMinNeighborsInRadius();
  }
}

export default RadiusOutlierRemoval;
