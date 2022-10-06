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

function collectTypeDefinition(moduleName) {
  const tscAlias = () => {
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
      renderChunk(code) {
        return `
${code}
  
export as namespace ${moduleName};`;
      },
    };
  };

  return [tscAlias(), dts()];
}

const constants = {
  __version__: pkg.version,
};

const commonPlugins = [
  alias({
    entries: { '@/': path.resolve(__dirname, 'src/') },
  }),
  bundleSize(),
  banner(),
];

const coreConfig = {
  input: 'src/core/index.ts',
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
    ...commonPlugins,
    replaceCode(),
    replace(constants),
    typescript({
      useTsconfigDeclarationDir: true,
    }),
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
};

const visualizationModules = ['PointCloudViewer'];
const visualizationConfig = visualizationModules.map((name) => {
  return {
    input: `src/visualization/${name}.ts`,
    output: [
      {
        file: `dist/visualization/${name}.esm.js`,
        format: 'esm',
      },
      {
        file: `dist/visualization/${name}.js`,
        format: 'iife',
        name,
        globals: {
          three: 'THREE',
        },
      },
      {
        file: `dist/visualization/${name}.min.js`,
        format: 'iife',
        name,
        globals: {
          three: 'THREE',
        },
        plugins: [terser()],
      },
    ],
    plugins: [
      ...commonPlugins,
      typescript({
        tsconfigOverride: {
          compilerOptions: {
            declaration: false,
          },
        },
      }),
    ],
    external: ['three'],
  };
});

const config = [
  coreConfig,
  ...visualizationConfig,
  {
    input: './dist/.types/core/index.d.ts',
    output: [{ file: 'dist/pcl.d.ts', format: 'es' }],
    plugins: collectTypeDefinition('PCL'),
  },
  ...visualizationModules.map((name) => ({
    input: `./dist/.types/visualization/${name}.d.ts`,
    output: [{ file: `dist/visualization/${name}.d.ts`, format: 'es' }],
    plugins: collectTypeDefinition(name),
  })),
];

export default config;
