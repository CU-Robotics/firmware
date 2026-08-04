#!/usr/bin/env bash
# install-ty.sh
#
# Installs tytools on Linux (Arch or Debian/Ubuntu), or tycmd on macOS.

set -euo pipefail

# --- detect distro family: prints "arch", "debian", or "unknown" ---
detect_distro() {
  if [[ -r /etc/os-release ]]; then
    . /etc/os-release
    case "${ID:-}" in
      arch|manjaro|endeavouros|arcolinux|artix|garuda) echo "arch"; return ;;
      ubuntu|debian|linuxmint|pop|raspbian|elementary|neon) echo "debian"; return ;;
    esac
    case "${ID_LIKE:-}" in
      *arch*)            echo "arch";   return ;;
      *debian*|*ubuntu*) echo "debian"; return ;;
    esac
  fi
  # or wtv package manager if they are niche
  if command -v pacman >/dev/null 2>&1; then echo "arch";   return; fi
  if command -v apt    >/dev/null 2>&1; then echo "debian"; return; fi
  echo "unknown"
}

OS="$(uname -s)"
echo "Detected OS: $OS"

if [[ "$OS" == "Linux" ]]; then
  DISTRO="$(detect_distro)"
  echo "--> Running Linux tytools installer (distro: $DISTRO)..."

  if [[ "$DISTRO" == "arch" ]]; then
    echo "--> Starting Arch tytools installer! :D"

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

  elif [[ "$DISTRO" == "debian" ]]; then
    echo "--> Starting Debian based tytools installer..."

    # 1. Ensure curl is present
    echo "Installing curl..."
    sudo apt update
    sudo apt install -y curl

    # 2. Prepare keyrings directory
    echo "Creating /etc/apt/keyrings with proper perm..."
    sudo mkdir -p -m0755 /etc/apt/keyrings

    # 3. Download Koromix key
    echo "Downloading Koromix GPG key..."
    sudo curl -fsSL \
      https://download.koromix.dev/debian/koromix-archive-keyring.gpg \
      -o /etc/apt/keyrings/koromix-archive-keyring.gpg

    # 4. Add repo to sources.list.d
    echo "Adding Koromix repo..."
    echo \
      "deb [signed-by=/etc/apt/keyrings/koromix-archive-keyring.gpg] \
https://download.koromix.dev/debian stable main" \
      | sudo tee /etc/apt/sources.list.d/koromix.dev-stable.list > /dev/null

    # 5. Update & install
    echo "Updating APT and installing tytools..."
    sudo apt update
    sudo apt install -y tytools

    echo "[OK] tytools installed!"

  else
    echo "[WARN] Unsupported Linux distro. Please install tytools manually, sorry!" >&2
    exit 1
  fi

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
