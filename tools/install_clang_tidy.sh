#!/usr/bin/env bash

set -euo pipefail

VERSION=18.1.8

Linux_x86_64_WHEEL="clang_tidy-18.1.8-py2.py3-none-manylinux_2_17_x86_64.manylinux2014_x86_64.whl"
Linux_x86_64_SHA256="c725a95c316d15df12f5bf7c1abfdf3cb9e4f3b9a9afb4cb6bb1e9df4e669493"
Darwin_arm64_WHEEL="clang_tidy-18.1.8-py2.py3-none-macosx_11_0_arm64.whl"
Darwin_arm64_SHA256="d635df55e45c9141b16f4d33ef785d3ec6944db66a43ece865981447460affc3"
Darwin_x86_64_WHEEL="clang_tidy-18.1.8-py2.py3-none-macosx_10_9_x86_64.whl"
Darwin_x86_64_SHA256="341f3bebe13ef31d6d3fb9a9fde52795557d7d67d7449e8ed98a36dfe29b7421"

OUTPUT=./tools/clang-tidy
STAMP="$OUTPUT/VERSION"

if [[ -x "$OUTPUT/bin/clang-tidy" && "$(cat "$STAMP" 2>/dev/null)" == "$VERSION" ]]; then
    exit 0
fi

platform="$(uname -s)_$(uname -m)"
wheel_var="${platform}_WHEEL"
sha_var="${platform}_SHA256"
wheel="${!wheel_var:-}"
sha="${!sha_var:-}"

if [[ -z "$wheel" ]]; then
    echo "No pinned clang-tidy build for $platform." >&2
    echo "Install clang-tidy $VERSION yourself and run with CLANG_TIDY=/path/to/clang-tidy." >&2
    exit 1
fi

for tool in curl unzip; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "$tool is required to install clang-tidy." >&2
        exit 1
    fi
done

echo "Installing clang-tidy $VERSION into $OUTPUT..."

tmp="$(mktemp -d "${TMPDIR:-/tmp}/clang-tidy.XXXXXX")"
trap 'rm -rf "$tmp"' EXIT

curl -fsSL --retry 3 -o "$tmp/clang-tidy.whl" \
    "https://files.pythonhosted.org/packages/py2.py3/c/clang-tidy/$wheel"

if command -v sha256sum >/dev/null 2>&1; then
    actual="$(sha256sum "$tmp/clang-tidy.whl" | cut -d' ' -f1)"
else
    actual="$(shasum -a 256 "$tmp/clang-tidy.whl" | cut -d' ' -f1)"
fi
if [[ "$actual" != "$sha" ]]; then
    echo "Checksum mismatch for $wheel (expected $sha, got $actual)." >&2
    exit 1
fi

unzip -q "$tmp/clang-tidy.whl" 'clang_tidy/data/*' -d "$tmp"
rm -rf "$OUTPUT"
mkdir -p "$OUTPUT"
mv "$tmp/clang_tidy/data/"* "$OUTPUT/"
chmod +x "$OUTPUT/bin/clang-tidy"
echo "$VERSION" > "$STAMP"

"$OUTPUT/bin/clang-tidy" --version
