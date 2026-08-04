#!/usr/bin/env bash
# install_arduino.sh
#
# Installs arduino-cli + the Teensy core.

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
  # pkg manager only 
  if command -v pacman >/dev/null 2>&1; then echo "arch";   return; fi
  if command -v apt    >/dev/null 2>&1; then echo "debian"; return; fi
  echo "unknown"
}

DISTRO="$(detect_distro)"
echo "Detected distro family: $DISTRO"

# verify that curl is installed
case "$DISTRO" in
  arch)
    sudo pacman -Sy --noconfirm
    sudo pacman -S --needed --noconfirm curl
    ;;
  debian)
    sudo apt update
    sudo apt install -y curl
    ;;
  *)
    echo "[WARN] Unsupported distro. Please install 'curl' manually, then re-run." >&2
    exit 1
    ;;
esac

# download arduino-cli's install script
curl -o arduino-install-thing.sh https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh
# make this executable
chmod +x arduino-install-thing.sh
# run the install script and place it in /usr/local/bin (env var)
sudo BINDIR=/usr/local/bin/ ./arduino-install-thing.sh
# clean up install script
rm arduino-install-thing.sh

# install the needed board/tools for Teensy
arduino-cli core install teensy:avr@1.59.0 --additional-urls "https://www.pjrc.com/teensy/package_teensy_index.json"

# install the compiler
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$SCRIPT_DIR/install_compiler.sh"
