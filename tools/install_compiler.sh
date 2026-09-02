#!/usr/bin/env bash
# install_compiler.sh
#
# Installs the gcc-arm-none-eabi toolchain into tools/compiler/arm-gnu-toolchain.
#
# The download is verified against a pinned SHA256 and only swapped into place
# once it has been extracted successfully, so a failed or interrupted run always
# leaves the previously installed toolchain untouched.

set -euo pipefail

# --- what to install -------------------------------------------------------
# To bump the toolchain, change VERSION and refresh the table below. Each row is
# "<uname -s> <uname -m>  <asset slug>  <sha256>". Arm publishes the checksums
# next to each tarball, e.g. <BASE_URL>/<asset>.tar.xz.sha256asc
VERSION="14.2.rel1"
BASE_URL="https://developer.arm.com/-/media/Files/downloads/gnu/${VERSION}/binrel"

# --- where to put it -------------------------------------------------------
# Derived from the script's own location so it works from any directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT="$SCRIPT_DIR/compiler"
DEST="$OUTPUT/arm-gnu-toolchain"

# --- pick the right tarball for this machine -------------------------------
OS="$(uname -s)"
ARCH="$(uname -m)"

# A shell running under Rosetta reports x86_64 even on Apple Silicon, which
# would silently install the (much slower) translated Intel toolchain.
# TODO: remove this when apple ends support for Intel-based apps in macOS 28
if [[ "$OS" == "Darwin" && "$ARCH" == "x86_64" ]] &&
   [[ "$(sysctl -n sysctl.proc_translated 2>/dev/null)" == "1" ]]; then
  echo "[WARN] Running under Rosetta; installing the native arm64 toolchain instead."
  ARCH="arm64"
fi

case "$OS $ARCH" in
  "Linux x86_64")
    SLUG="x86_64-arm-none-eabi"
    SHA256="62a63b981fe391a9cbad7ef51b17e49aeaa3e7b0d029b36ca1e9c3b2a9b78823" ;;
  "Linux aarch64")
    SLUG="aarch64-arm-none-eabi"
    SHA256="87330bab085dd8749d4ed0ad633674b9dc48b237b61069e3b481abd364d0a684" ;;
  "Darwin x86_64")
    SLUG="darwin-x86_64-arm-none-eabi"
    SHA256="2d9e717dd4f7751d18936ae1365d25916534105ebcb7583039eff1092b824505" ;;
  "Darwin arm64")
    SLUG="darwin-arm64-arm-none-eabi"
    SHA256="c7c78ffab9bebfce91d99d3c24da6bf4b81c01e16cf551eb2ff9f25b9e0a3818" ;;
  *)
    echo "[ERROR] Unsupported platform: $OS $ARCH" >&2
    echo "Arm ships this toolchain for Linux (x86_64, aarch64) and macOS (x86_64, arm64) only." >&2
    echo "Install it manually into $DEST" >&2
    exit 1
    ;;
esac

ASSET="arm-gnu-toolchain-${VERSION}-${SLUG}.tar.xz"
URL="$BASE_URL/$ASSET"

# macOS ships shasum, most Linux distros ship sha256sum. Resolve it up front so
# a machine that has neither fails before downloading 128MB it cannot verify.
if command -v shasum >/dev/null 2>&1; then
  SHA_CMD=(shasum -a 256)
elif command -v sha256sum >/dev/null 2>&1; then
  SHA_CMD=(sha256sum)
else
  echo "[ERROR] Neither shasum nor sha256sum is available; cannot verify the download." >&2
  exit 1
fi

# --- staging area ----------------------------------------------------------
# Kept inside $OUTPUT (already gitignored, and on the same filesystem as $DEST
# so the final swap is a rename rather than a copy of ~1GB).
mkdir -p "$OUTPUT"
WORK="$(mktemp -d "$OUTPUT/.install.XXXXXX")"

# If anything below fails after the old toolchain has been moved aside, put it
# back before clearing the staging area.
cleanup() {
  if [[ -d "$WORK/previous" && ! -e "$DEST" ]]; then
    echo "Restoring the previous toolchain..." >&2
    mv "$WORK/previous" "$DEST"
  fi
  rm -rf "$WORK"
}
trap cleanup EXIT

# --- download --------------------------------------------------------------
# -f so an HTTP error page is never written out as a "tarball", -C - so a retry
# resumes instead of restarting the transfer from zero.
echo "Downloading $ASSET ..."
if [[ -t 2 ]]; then
  PROGRESS="--progress-bar"   # interactive: one bar
else
  PROGRESS="--silent"         # piped/CI: no per-update spam, -S keeps errors
fi
curl -fL "$PROGRESS" --show-error --retry 3 --retry-delay 2 -C - -o "$WORK/$ASSET" "$URL"

# --- verify ----------------------------------------------------------------
echo "Verifying checksum..."
ACTUAL="$("${SHA_CMD[@]}" "$WORK/$ASSET" | awk '{print $1}')"
if [[ "$ACTUAL" != "$SHA256" ]]; then
  echo "[ERROR] Checksum mismatch for $ASSET" >&2
  echo "  expected: $SHA256" >&2
  echo "  actual:   $ACTUAL" >&2
  echo "The download was corrupted or tampered with. Nothing was installed." >&2
  exit 1
fi

# --- extract ---------------------------------------------------------------
echo "Extracting the compiler..."
mkdir -p "$WORK/extract"
tar -xf "$WORK/$ASSET" -C "$WORK/extract"
rm -f "$WORK/$ASSET"

# The tarball should contain exactly one top-level directory.
shopt -s nullglob
EXTRACTED=("$WORK"/extract/*/)
shopt -u nullglob
if [[ ${#EXTRACTED[@]} -ne 1 ]]; then
  echo "[ERROR] Expected one directory in $ASSET, found ${#EXTRACTED[@]}." >&2
  exit 1
fi

# --- swap into place -------------------------------------------------------
# Only now is the old toolchain touched, and it is moved aside rather than
# deleted so the cleanup trap can put it back if this fails.
echo "Installing to $DEST ..."
if [[ -e "$DEST" ]]; then
  mv "$DEST" "$WORK/previous"
fi
mv "${EXTRACTED[0]%/}" "$DEST"

# Discard the old toolchain now rather than leaving a multi-second delete
# running under the trap after the success message.
echo "Cleaning up..."
trap - EXIT
rm -rf "$WORK"

echo "[OK] arm-none-eabi toolchain $VERSION installed at $DEST"
