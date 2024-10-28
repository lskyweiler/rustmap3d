#!/bin/bash
cd rust/rustmap3d_py 

# build many linux
# https://github.com/rust-lang/cargo/issues/7154
RUSTFLAGS="-C target-feature=-crt-static" /home/rust/.venv/bin/python -m maturin build --release -i 3.8 3.9 3.10 3.11 3.12 3.13
# build windows
/home/rust/.venv/bin/python -m maturin build --release -i 3.8 3.9 3.10 3.11 3.12 3.13  --target x86_64-pc-windows-gnu

# https://doc.rust-lang.org/nightly/rustc/platform-support.html
/home/rust/.venv/bin/python -m maturin build --release -i 3.8 3.9 3.10 3.11 3.12 3.13 --target aarch64-unknown-linux-gnu
# /home/rust/.venv/bin/python -m maturin build --release -i 3.8 3.9 3.10 3.11 3.12 --target aarch64-apple-darwin
# /home/rust/.venv/bin/python -m maturin build --release -i 3.8 3.9 3.10 3.11 3.12 --target x86_64-apple-darwin