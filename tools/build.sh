#!/usr/bin/env bash
# Build WASMLib: dist/wasmlib.mjs + dist/wasmlib.wasm (needs Emscripten on PATH: source emsdk_env.sh first).
#   tools/build.sh            the JavaScript module
#   tools/build.sh test       also upstream's SM_API test suite, run in Node (from upstream/tests/SM_API_test)
#   tools/build.sh native     the native reference build of the same test suite, and run it
set -euo pipefail
cd "$(dirname "$0")/.."
git submodule update --init --depth 1 upstream
python3 tools/gen_smapi.py
case "${1:-}" in
  native)
    cmake -B build-native -G Ninja -DCMAKE_BUILD_TYPE=Release
    ninja -C build-native sm_api_test
    (cd upstream/tests/SM_API_test && time ../../../build-native/sm_api_test | tail -4) ;;
  *)
    emcmake cmake -B build-wasm -G Ninja -DCMAKE_BUILD_TYPE=Release
    ninja -C build-wasm wasmlib
    if [ "${1:-}" = "test" ]; then
      ninja -C build-wasm sm_api_test
      (cd upstream/tests/SM_API_test && time node ../../../build-wasm/sm_api_test.js | tail -4)
    fi ;;
esac
ls -la dist/
