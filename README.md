# rustmap3d

![Rust](https://img.shields.io/badge/Rust-%23000000.svg?&logo=rust&logoColor=white&color=000)
![Python](https://img.shields.io/badge/Python_3.8_|_3.9_|_3.10_|_3.11_|_3.12_|_3.13-blue?logo=python&logoColor=fff)
[![PyPI](https://img.shields.io/badge/PyPI-3775A9?logo=pypi&logoColor=fff)](https://gitlab.sdo.psdo.leidos.com/alphamosaic/registry/-/packages)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)

<p align="center"><img src="docs/rustmap3d.logo.svg" width="300px" height="300px"/></p>

**Simple, fast, and ergonomic geodetic coordinate conversions**

## Benchmarks

Compared to pure python equivalent code in [pymap3d](https://github.com/geospace-code/pymap3d)

- *50x* faster for lla2ecef
- *400x* faster for ecef2lla

<p align="center"><img src="docs/benchmarks.svg"></p>

```bash
# Run benchmarks
uv run pytest --benchmark-histogram="./docs/benchmarks" bench/
```
