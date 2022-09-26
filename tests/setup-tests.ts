import { initPCL } from './common';

beforeAll(async () => {
  global.pcl = await initPCL();
});
