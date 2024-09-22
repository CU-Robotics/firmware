# if this is arch, we need to manually install the compiler

curl_output=arm-none-eabi-11.3.tar.xz
compiler_dest=/packages/teensy/tools/teensy-compile/11.3.1/arm
arduino_path=""

# fill in the arduino path based on OS
if [ "$(uname -s)" = "Darwin" ]; then
    # no quotes because it will encode '~' as an actual folder rather than home
    arduino_path=~/Library/Arduino15
elif [ "$(uname -s)" = "Linux" ]; then
    # no quotes because it will encode '~' as an actual folder rather than home
    arduino_path=~/.arduino15
else 
    echo "Unsupported Operating System: $(uname -s)"
    exit 1
fi;

# if this is aarch64 (arm), install the compiler manually
if [ "$(arch)" = "aarch64" ]; then
    # if the destination folder already has contents, erase them
    rm -rf $arduino_path$compiler_dest
	# create the destination folder for the compiler binaries and supporting files	
	mkdir -p $arduino_path$compiler_dest
	# download the compiler
	curl https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/11.3.rel1/binrel/arm-gnu-toolchain-11.3.rel1-aarch64-arm-none-eabi.tar.xz -o $curl_output
	# extract the tar
	tar xf "$curl_output"
    	# store the current directory so we can go back eventually
    	curr_dir=$(pwd)
	# go into the compiler folder
	cd arm-gnu-toolchain-11.3.rel1-aarch64-arm-none-eabi/
	# copy the compiler binaries and supporting files into the destination
	cp -r * $arduino_path$compiler_dest
	# go back and clean up the extracted tar file
	cd $curr_dir
	rm -rf arm-gnu-toolchain-11.3.rel1-aarch64-arm-none-eabi/
	rm -rf $curl_output

    # exit so we dont install arduino at all
    exit 0
fi;

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
