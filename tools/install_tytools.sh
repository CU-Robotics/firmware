#!/usr/bin/env bash
# install-ty.sh
#
# Installs tytools on Linux, or tycmd on macOS.

set -euo pipefail

OS="$(uname -s)"
echo "Detected OS: $OS"

if [[ "$OS" == "Linux" ]]; then
  echo "--> Running Arch tytools installer! :D"

  # 1. Ensure build tools (base-devel), git, and curl are present
  echo "Ensuring base-devel, git, and curl are up to date..."
  sudo pacman -Sy --noconfirm
  sudo pacman -S --needed --noconfirm base-devel git curl

  # 2. Prepare a temporary build directory for AUR package
  echo "Creating temp build directory..."   
  BUILD_DIR="$(mktemp -d)"

  # 3. Clone the tytools AUR package
  echo "Downloading tytools AUR pkg..."
  git clone https://aur.archlinux.org/tytools.git "$BUILD_DIR/tytools"

  # 4. Enter the cloned package directory
  echo "Entering tytools build dir..." 
  cd "$BUILD_DIR/tytools"

  # 5. Build and install tytools with makepkg
  echo "Building..."
  makepkg -si --noconfirm
  cd - >/dev/null && rm -rf "$BUILD_DIR"

  echo "[DONE] tytools installed! yay! :3"

elif [[ "$OS" == "Darwin" ]]; then
  echo "--> Running macOS installer (tycmd)..."

  REPO_URL="https://github.com/CU-Robotics/tycmd.git"
  BIN_NAME="tycmd"
  DEST_DIR="/usr/local/bin"
  TMP_DIR="$(mktemp -d)"

  echo "Cloning ${REPO_URL} into ${TMP_DIR}..."
  git clone "${REPO_URL}" "${TMP_DIR}"

  # Verify file exists (may not be executable yet)
  if [[ ! -e "${TMP_DIR}/${BIN_NAME}" ]]; then
    echo "Error: '${BIN_NAME}' not found in repo root."
    exit 1
  fi

  echo "Moving '${BIN_NAME}' to ${DEST_DIR} (requires sudo)..."
  sudo mv "${TMP_DIR}/${BIN_NAME}" "${DEST_DIR}/"

  echo "Setting executable permissions..."
  sudo chmod +x "${DEST_DIR}/${BIN_NAME}"

  echo "Cleaning up..."
  rm -rf "${TMP_DIR}"

  echo "[OK] '${BIN_NAME}' installed to ${DEST_DIR}/${BIN_NAME}."

else
  echo "[WARN]  Unsupported OS: $OS"
  echo "Please install manually."
  exit 1
fi
