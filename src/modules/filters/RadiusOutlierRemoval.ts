import FilterBase from './FilterBase';

class RadiusOutlierRemoval extends FilterBase {
  constructor(extractRemovedIndices = false) {
    super(new Module.RadiusOutlierRemoval(extractRemovedIndices));
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
