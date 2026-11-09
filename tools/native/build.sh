#!/usr/bin/env bash
# Build the native replays (tools/native/replay.cpp) against the native SMLib (cmake -B build-native ... first).
#   tools/native/build.sh [extra flags, e.g. -g for callgrind's source lines]
set -euo pipefail
cd "$(dirname "$0")/../.."
[ -f build-native/libsmapi.a ] || { cmake -B build-native -G Ninja -DCMAKE_BUILD_TYPE=Release && ninja -C build-native smapi; }
g++ -std=c++17 -O2 -w -DNDEBUG -DNOMINMAX "$@" -Itools/native -Ibind -Ismlib/source/SMLib/inc -Ismlib/source/SM_API/inc \
  tools/native/replay.cpp build-native/libsmapi.a build-native/libsmlib.a -o build-native/replay
echo build-native/replay
