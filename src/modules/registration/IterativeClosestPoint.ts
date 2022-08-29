class IterativeClosestPoint {
  public native: any;

  constructor() {
    this.native = new __PCLCore__.IterativeClosestPoint();
  }

  public setInputSource(fieldName: string) {
    return this.native.setInputSource(fieldName);
  }

  public setInputTarget(fieldName: string) {
    return this.native.setInputTarget(fieldName);
  }

  public getFinalTransformation(): string | null {
    return this.native.getFinalTransformation() as string | null;
  }

  public hasConverged() {
    return this.native.hasConverged();
  }

  public align() {
    return this.native.align();
  }
}

export default IterativeClosestPoint;
