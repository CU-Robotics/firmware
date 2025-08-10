#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="$ROOT/build"

mkdir -p "$BUILD"

cmake -S "$ROOT" \
      -B "$BUILD" \
      -DCMAKE_TOOLCHAIN_FILE="$ROOT/cmake/toolchain-teensy41.cmake" \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo | cat

ln -sf "$BUILD/compile_commands.json" "$ROOT/compile_commands.json"

echo "[CMake configured] compile_commands.json linked at repo root"




