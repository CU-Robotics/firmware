RED='\033[0;31m'
NC='\033[0m' # No Color

if command -v rustc &> /dev/null
then
    echo "Rust is installed"
else

    echo -e "${RED}It looks like rust is not installed, attempting automatic installation (see https://www.rust-lang.org/tools/install)${NC}"
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
    echo "Rust is not installed"
fi

echo "Compiling the project. This may take a while the first time..."
cargo build --manifest-path tools/waggle-interceptor/Cargo.toml --release
./tools/waggle-interceptor/target/release/waggle-interceptor make upload
