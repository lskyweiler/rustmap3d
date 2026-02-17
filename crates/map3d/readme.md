# map3d

[![License](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/lskyweiler/rustmap3d#license)
[![crates.io](https://img.shields.io/crates/v/map3d.svg)](https://crates.io/crates/map3d)

Rust geodetic coordinate transformation library

> This library is the standalone rust backend for [rustmap3d](https://github.com/lskyweiler/rustmap3d)

Similar to [map_3d](https://crates.io/crates/map_3d) except for the following:

1. all functions use glam vectors
2. rotations are exposed (both quaternions and 3x3 DCMs)
