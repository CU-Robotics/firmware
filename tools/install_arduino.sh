# verify that curl is installed
sudo apt update
sudo apt install -y curl

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

# install the compiler if we're running on arm (aarch64)
./tools/install_compiler.sh
