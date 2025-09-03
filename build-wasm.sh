#!/bin/bash

# Source Emscripten SDK environment
source ../emsdk/emsdk_env.sh

mkdir -p build-wasm
cd build-wasm

emcmake cmake ../wrappers/wasm \
  -DCMAKE_BUILD_TYPE=Release \
  -DZXING_WRITERS=ON \
  -DZXING_READERS=ON \
  -DZXING_EXAMPLES=OFF

emmake make -j$(nproc)
