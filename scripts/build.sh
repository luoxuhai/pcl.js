#!/bin/bash

PROJECT_ROOT=$(pwd)
WASM_OUT_DIR=${PROJECT_ROOT}/dist

# Build embind

EMBIND_BUILD_DIR=${PROJECT_ROOT}/src/bind/build

echo $(pwd)

mkdir -p ${EMBIND_BUILD_DIR}
cd ${EMBIND_BUILD_DIR}

if [[ $1 = "Release" ]]; then
  BUILD_TYPE=Release
else
  BUILD_TYPE=Debug
fi

EMFLAGS=(
  -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
  # PCL Directory
  -DPCL_ROOT=${PROJECT_ROOT}/core/pcl
)

emcmake cmake ${EMFLAGS[@]} ..
emmake make -j

mkdir -p ${WASM_OUT_DIR}
cp pcl-core.wasm ${WASM_OUT_DIR}
