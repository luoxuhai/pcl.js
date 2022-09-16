import { initPCL } from './common';

beforeAll(async () => {
  if (!(window as any).pcl) {
    (window as any).pcl = await initPCL();
  }
});
