#!/bin/bash

set -e

git clone https://github.com/stm32-rs/stm32-rs
cd stm32-rs
cargo install svd2rust
rustup component add rustfmt
pip install --user pyyaml
cd svd
./extract.sh
cd ..
make patch
make -j$(nproc) svd2rust
cd ..

sed -i 's|/your/path/here/||' Cargo.toml
