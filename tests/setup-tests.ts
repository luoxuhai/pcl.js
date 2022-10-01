import { initPCL } from './common';
import { TextDecoder } from 'util';

if (!window.TextDecoder) {
  window.TextDecoder = TextDecoder as any;
}

beforeAll(async () => {
  global.pcl = await initPCL();
});
