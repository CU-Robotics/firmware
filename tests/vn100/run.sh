#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
mkdir -p build/tests
# Match ARM's unsigned char, and adapt the existing ASCII parser to the host
# libc's const-correct strchr overload. The production binary parser is unchanged.
sed '/^[[:space:]]*char \*data_start = strchr/s/char \*/const char */' \
    libraries/vn100/protocol/msg_parsing.cpp > build/tests/vn100_msg_parsing.cpp
"${CXX:-c++}" -std=c++17 -funsigned-char -Wno-format \
    -Itests/vn100/stubs -Isrc -Ilibraries -Ilibraries/vn100/protocol \
    tests/vn100/test.cpp src/sensors/vn100.cpp \
    libraries/vn100/protocol/serial.cpp \
    libraries/vn100/protocol/bin_parsing.cpp \
    build/tests/vn100_msg_parsing.cpp \
    libraries/vn100/protocol/msg_creation.cpp \
    -o build/tests/vn100
build/tests/vn100
