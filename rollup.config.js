import typescript from 'rollup-plugin-typescript2';
import { terser } from 'rollup-plugin-terser';

import pkg from './package.json';

export default {
  input: 'src/index.ts', // our source file
  output: [
    {
      file: pkg.main,
      format: 'cjs',
    },
    {
      file: pkg.module,
      format: 'es', // the preferred format
    },
    {
      file: pkg.browser,
      format: 'iife',
      name: 'MyPackage', // the global which can be used in a browser
    },
  ],
  plugins: [
    typescript({
      clean: true,
      tsconfig: './tsconfig.json',
    }),
    terser(),
  ],
};
