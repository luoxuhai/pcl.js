import typescript from 'rollup-plugin-typescript2';
import { terser } from 'rollup-plugin-terser';
import dts from 'rollup-plugin-dts';
import bundleSize from 'rollup-plugin-bundle-size';
import copy from 'rollup-plugin-copy';
import replace from '@rollup/plugin-replace';
import serve from 'rollup-plugin-serve';
import alias from '@rollup/plugin-alias';
import path from 'path';
import { exec } from 'child_process';

import pkg from './package.json';

function banner() {
  return {
    renderChunk(code) {
      return `/**
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
      return (
        code
          .replace('assert(!flags, flags);', '')
          // test environment
          .replace(
            "ENVIRONMENT_IS_NODE = typeof process == 'object' && typeof process.versions == 'object' && typeof process.versions.node == 'string'",
            'ENVIRONMENT_IS_NODE = false',
          )
      );
    },
  };
}

function tscAlias() {
  return {
    name: 'tsAlias',
    buildStart: () => {
      return new Promise((resolve, reject) => {
        exec('tsc-alias', function callback(error, stdout, stderr) {
          if (stderr || error) {
            reject(stderr || error);
          } else {
            resolve(stdout);
          }
        });
      });
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
        globals: {
          three: 'THREE',
        },
      },
      {
        file: `dist/pcl.min.js`,
        format: 'iife',
        name: 'PCL',
        globals: {
          three: 'THREE',
        },
        plugins: [
          replaceCode(),
          terser({
            compress: {
              drop_console: true,
            },
          }),
        ],
      },
    ],
    plugins: [
      alias({
        entries: { '@/': path.resolve(__dirname, 'src/') },
      }),
      replaceCode(),
      replace({
        __version__: pkg.version,
      }),
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
      process.env.ENV === 'development' &&
        serve({
          contentBase: 'dist',
          port: 4321,
          headers: {
            'Access-Control-Allow-Origin': '*',
          },
        }),
    ],
    external: ['three'],
  },
  {
    input: './dist/types/.temp/index.d.ts',
    output: [{ file: 'dist/types/pcl.d.ts', format: 'es' }],
    plugins: [tscAlias(), dts()],
  },
];

export default config;
