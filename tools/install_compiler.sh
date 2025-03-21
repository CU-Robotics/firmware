# download links for gcc-arm-none-eabi-14.2.rel1
x86_64_Linux="https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz"
AArch64_Linux="https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-aarch64-arm-none-eabi.tar.xz"
x86_64_MacOS="https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-darwin-x86_64-arm-none-eabi.tar.xz"
Arm64_MacOS="https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi.tar.xz"

# where to store the compiler
OUTPUT=./tools/compiler

TAR_NAME=./arm-gnu-toolchain-14.2.rel1.tar.xz

# remove the old compiler
rm -rf $OUTPUT

# create a directory to store the compiler
mkdir -p $OUTPUT

# install the correct compiler based on the OS and architecture
if [[ "$(uname -s)" == "Linux" ]]; then
    if [[ $(uname -m) == "x86_64" ]]; then
        wget $x86_64_Linux -O $TAR_NAME
    elif [[ $(uname -m) == "aarch64" ]]; then
        wget $AArch64_Linux -O $TAR_NAME
    fi
elif [[ "$(uname -s)" == "Darwin" ]]; then
    if [[ $(uname -m) == "x86_64" ]]; then
        wget $x86_64_MacOS -O $TAR_NAME
    elif [[ $(uname -m) == "arm64" ]]; then
        wget $Arm64_MacOS -O $TAR_NAME
    fi
fi

# extract the compiler
echo "Extracting the compiler..."
tar -xf $TAR_NAME -C $OUTPUT
mv $OUTPUT/arm-gnu-toolchain* $OUTPUT/arm-gnu-toolchain

# remove the downloaded tar files
echo "Cleaning up..."
rm -f $TAR_NAME
