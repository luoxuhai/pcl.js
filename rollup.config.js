import typescript from 'rollup-plugin-typescript2';
import { terser } from 'rollup-plugin-terser';
import dts from 'rollup-plugin-dts';
import bundleSize from 'rollup-plugin-bundle-size';

import pkg from './package.json';

function banner() {
  return {
    renderChunk(code) {
      return `/**
 * @license
 * PCL.js v${pkg.version} Copyright ${new Date().getFullYear()} ${
        pkg.author
      } and contributors
 * SPDX-License-Identifier: MIT
 */
${code}`;
    },
  };
}

const config = [
  {
    input: 'src/index.ts',
    output: [
      {
        file: pkg.main,
        format: 'cjs',
      },
      {
        file: pkg.module,
        format: 'esm',
      },
      {
        file: pkg.browser,
        format: 'umd',
        name: 'PCL',
      },
    ],
    plugins: [
      typescript({
        useTsconfigDeclarationDir: true,
        tsconfig: './tsconfig.json',
      }),
      terser({
        compress: false,
        format: {
          comments: false,
        },
      }),
      bundleSize(),
      banner(),
    ],
  },
  {
    input: './dist/types/index.d.ts',
    output: [{ file: 'dist/pcl.d.ts', format: 'es' }],
    plugins: [dts()],
  },
];

export default config;
