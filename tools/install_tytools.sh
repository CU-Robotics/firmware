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

    # 1. Ensure build tools (base-devel), git, and curl are present (IF YOU HAVE PROBLEM WITH SYU INSTALL MANUALLY!!!)
    echo "Ensuring base-devel, git, and curl are up to date..." 
    sudo pacman -Syu --needed --noconfirm base-devel git curl

    # 2. Prepare a temporary build directory for AUR package
    echo "Creating temp build directory..."
    BUILD_DIR="$(mktemp -d)"
    # Clean up even if the clone or build fails partway through
    trap 'rm -rf "$BUILD_DIR"' EXIT

    # 3. Clone the tytools AUR package
    echo "Downloading tytools AUR pkg..."
    git clone https://aur.archlinux.org/tytools.git "$BUILD_DIR/tytools"

    # 4. Build and install tytools with makepkg, in a subshell so we do not
    #    have to cd back afterwards
    echo "Building..."
    ( cd "$BUILD_DIR/tytools" && makepkg -si --noconfirm )

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

  # tycmd is a prebuilt binary installed with sudo, so it is pinned to one
  # commit and verified rather than tracking whatever is on main. To bump it,
  # pick a commit and record its hash:
  #   curl -fsSL https://raw.githubusercontent.com/CU-Robotics/tycmd/<sha>/tycmd | shasum -a 256
  TYCMD_COMMIT="5a416e8adfa0c454948f53bd801b3ce3e7f30f9c"
  TYCMD_SHA256="00593e4d53672bf3ae86d30bb8eead3f4c8f671480f706a33129d2368952444a"
  TYCMD_URL="https://raw.githubusercontent.com/CU-Robotics/tycmd/${TYCMD_COMMIT}/tycmd"

  BIN_NAME="tycmd"
  DEST_DIR="/usr/local/bin"
  TMP_DIR="$(mktemp -d)"
  trap 'rm -rf "$TMP_DIR"' EXIT

  echo "Downloading ${BIN_NAME} (${TYCMD_COMMIT:0:7})..."
  curl -fL --retry 3 --retry-delay 2 -o "${TMP_DIR}/${BIN_NAME}" "$TYCMD_URL"

  echo "Verifying checksum..."
  ACTUAL="$(shasum -a 256 "${TMP_DIR}/${BIN_NAME}" | awk '{print $1}')"
  if [[ "$ACTUAL" != "$TYCMD_SHA256" ]]; then
    echo "[ERROR] Checksum mismatch for ${BIN_NAME}" >&2
    echo "  expected: $TYCMD_SHA256" >&2
    echo "  actual:   $ACTUAL" >&2
    echo "The download was corrupted or the pinned commit changed. Nothing was installed." >&2
    exit 1
  fi

  # macOS does not create /usr/local/bin, and Homebrew on Apple Silicon uses
  # /opt/homebrew, so a clean Mac may not have it yet.
  echo "Installing '${BIN_NAME}' to ${DEST_DIR} (requires sudo)..."
  sudo mkdir -p "${DEST_DIR}"
  sudo install -m 0755 "${TMP_DIR}/${BIN_NAME}" "${DEST_DIR}/${BIN_NAME}"

  echo "[OK] '${BIN_NAME}' installed to ${DEST_DIR}/${BIN_NAME}."

else
  echo "[WARN]  Unsupported OS: $OS"
  echo "Please install manually."
  exit 1
fi
