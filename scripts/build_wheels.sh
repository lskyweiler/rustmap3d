#!/bin/bash
cd rust/rustmap3d_py 
rustup target add x86_64-pc-windows-gnu
# build many linux
/home/rust/.venv/bin/python -m maturin build --release -i 3.8 3.9 3.10 3.11 3.12
# build windows
/home/rust/.venv/bin/python -m maturin build --release -i 3.8 3.9 3.10 3.11 3.12 --target x86_64-pc-windows-gnu