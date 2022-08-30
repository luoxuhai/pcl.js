import typescript from 'rollup-plugin-typescript2';
import { terser } from 'rollup-plugin-terser';
import dts from 'rollup-plugin-dts';
import bundleSize from 'rollup-plugin-bundle-size';
import copy from 'rollup-plugin-copy';

import pkg from './package.json';

function banner() {
  return {
    renderChunk(code) {
      return `/*!
* pcl.js - v${pkg.version}
*
* Copyright (c) ${new Date().getFullYear()} pcl.js Authors
* SPDX-License-Identifier: MIT
*/

${code}`;
    },
  };
}

function replaceCode() {
  return {
    renderChunk(code) {
      return code.replace('assert(!flags, flags);', '');
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
        file: `dist/pcl.js`,
        format: 'iife',
        name: 'PCL',
      },
      {
        file: `dist/pcl.min.js`,
        format: 'iife',
        name: 'PCL',
        plugins: [replaceCode(), terser()],
      },
    ],
    plugins: [
      replaceCode(),
      typescript({
        useTsconfigDeclarationDir: true,
        tsconfig: './tsconfig.json',
      }),
      bundleSize(),
      banner(),
      copy({
        targets: [
          {
            src: 'src/bind/build/pcl-core.wasm',
            dest: 'dist',
          },
        ],
      }),
    ],
  },
  {
    input: './dist/types/.temp/index.d.ts',
    output: [{ file: 'dist/types/pcl.d.ts', format: 'es' }],
    plugins: [dts()],
  },
];

export default config;
