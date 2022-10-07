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
      return code.replace('assert(!flags, flags);', '');
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

const browserCoreConfig = {
  input: 'src/core/browser.ts',
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
        contentBase: './',
        port: 4321,
        headers: {
          'Access-Control-Allow-Origin': '*',
        },
      }),
  ],
};

const nodeCoreConfig = {
  input: 'src/core/node.ts',
  output: [
    {
      file: pkg.exports['.'].node.require,
      format: 'cjs',
    },
    {
      file: pkg.exports['.'].node.import,
      format: 'esm',
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
          src: 'src/bind/build/pcl-core.node.wasm',
          dest: 'dist/node',
        },
      ],
    }),
  ],
};

const visualizationModules = ['PointCloudViewer'];
const globals = {
  three: 'THREE',
  'three/examples/jsm/controls/OrbitControls': 'THREE',
  'three/examples/jsm/loaders/PCDLoader': 'THREE',
};
const external = [
  'three',
  'three/examples/jsm/controls/OrbitControls',
  'three/examples/jsm/loaders/PCDLoader',
];
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
        globals,
      },
      {
        file: `dist/visualization/${name}.min.js`,
        format: 'iife',
        name,
        globals,
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
    external,
  };
});

const config = [
  browserCoreConfig,
  nodeCoreConfig,
  ...visualizationConfig,
  {
    input: './dist/.types/core/browser.d.ts',
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
