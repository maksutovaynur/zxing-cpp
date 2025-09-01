#!/bin/bash
mkdir -p build-wasm
cd build-wasm

emcmake cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WRITERS=ON \
  -DBUILD_READERS=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_BLACKBOX_TESTS=OFF \
  -DBUILD_UNIT_TESTS=OFF

emmake make -j$(nproc)
