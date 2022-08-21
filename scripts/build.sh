#!/usr/bin/env bash

PROJECT_ROOT=$(pwd)
WASM_OUT_DIR=${PROJECT_ROOT}/dist

# # Build PCL
# cd $(dirname $0)/core/pcl
# build-wasm.sh

# Build embind

EMBIND_BUILD_DIR=${PROJECT_ROOT}/src/embind/build

echo $(pwd)

mkdir -p ${EMBIND_BUILD_DIR}
cd ${EMBIND_BUILD_DIR}

if [ $1 == Release ]
  then
    BUILD_TYPE=Release
  else
    BUILD_TYPE=Debug
fi

emcmake cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ..
emmake make -j

mkdir -p ${WASM_OUT_DIR}
cp pcl-core.wasm ${WASM_OUT_DIR}
