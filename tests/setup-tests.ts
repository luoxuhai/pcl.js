import { initPCL } from './utils';
import { TextDecoder } from 'util';

if (!window.TextDecoder) {
  window.TextDecoder = TextDecoder as any;
}

global.ROOT_DIR = process.env.PWD;

beforeAll(async () => {
  global.pcl = await initPCL();
});
