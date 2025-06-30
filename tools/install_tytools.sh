#!/usr/bin/env bash
# install-ty.sh
#
# Installs tytools on Linux, or tycmd on macOS.

set -euo pipefail

OS="$(uname -s)"
echo "Detected OS: $OS"

if [[ "$OS" == "Linux" ]]; then
  echo "→ Running Linux tytools installer…"

  # 1. Ensure curl is present
  echo "Installing curl…"
  sudo apt update
  sudo apt install -y curl

  # 2. Prepare keyrings directory
  echo "Creating /etc/apt/keyrings with proper perm…"
  sudo mkdir -p -m0755 /etc/apt/keyrings

  # 3. Download Koromix key
  echo "Downloading Koromix GPG key…"
  sudo curl -fsSL \
    https://download.koromix.dev/debian/koromix-archive-keyring.gpg \
    -o /etc/apt/keyrings/koromix-archive-keyring.gpg

  # 4. Add repo to sources.list.d
  echo "Adding Koromix repo…"
  echo \
    "deb [signed-by=/etc/apt/keyrings/koromix-archive-keyring.gpg] \
https://download.koromix.dev/debian stable main" \
    | sudo tee /etc/apt/sources.list.d/koromix.dev-stable.list > /dev/null

  # 5. Update & install
  echo "Updating APT and installing tytools…"
  sudo apt update
  sudo apt install -y tytools

  echo "✅ tytools installed!"

elif [[ "$OS" == "Darwin" ]]; then
  echo "→ Running macOS installer (tycmd)…"

  REPO_URL="https://github.com/guywithhat99/tycmd.git"
  BIN_NAME="tycmd"
  DEST_DIR="/usr/local/bin"
  TMP_DIR="$(mktemp -d)"

  echo "🔄Cloning ${REPO_URL} into ${TMP_DIR}…"
  git clone "${REPO_URL}" "${TMP_DIR}"

  # Verify binary exists
  if [[ ! -x "${TMP_DIR}/${BIN_NAME}" ]]; then
    echo "❌ Error: '${BIN_NAME}' not found or not executable in repo root."
    exit 1
  fi

  echo "⬆️ Moving '${BIN_NAME}' to ${DEST_DIR} (requires sudo)…"
  sudo mv "${TMP_DIR}/${BIN_NAME}" "${DEST_DIR}/"

  echo "🔒Setting executable permissions…"
  sudo chmod +x "${DEST_DIR}/${BIN_NAME}"

  echo "🧹Cleaning up…"
  rm -rf "${TMP_DIR}"

  echo "✅ '${BIN_NAME}' installed to ${DEST_DIR}/${BIN_NAME}."

else
  echo "⚠️  Unsupported OS: $OS"
  echo "Please install manually."
  exit 1
fi