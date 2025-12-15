# rustmap3d

![Python](https://img.shields.io/badge/Python_3.8_|_3.9_|_3.10_|_3.11_|_3.12_|_3.13-blue?logo=python&logoColor=fff)
![Rust](https://img.shields.io/badge/Rust-%23000000.svg?&logo=rust&logoColor=white&color=000)
[![PyPI](https://img.shields.io/badge/PyPI-3775A9?logo=pypi&logoColor=fff)](https://gitlab.sdo.psdo.leidos.com/alphamosaic/registry/-/packages)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)

<p align="center"><img src="docs/rustmap3d.logo.svg" width="300px" height="300px"/></p>

**Simple, fast, and ergonomic geodetic coordinate conversions**

```python
import rustmap3d


# geodetic conversions
lla = rustmap3d.ecef2lla(x, y, z)
ecef = rustmap3d.lla2ecef(lat, lon, alt)
...

# local conversions
ecef = enu2ecef(e, n, u, lat_ref, lon_ref)
ned = ned2ecef(n, e, d, lat_ref, lon_ref)
# enu, ned, aer
...  

# local rotations
enu_quat = enu2ecef_quat(lat, lon)  # quaternion that rotates a local enu vector to ecef
enu_dcm = enu2ecef_dcm(lat, lon) 
# enu, ned, aer
...  

# distance functions
lat, lon = vincenty_direct(lat_deg, lon_deg, range_m, bearing_deg)
range_m, bearing_ab, bearing_ba = vincenty_inverse(lat_a, lon_a, lat_b, lon_b)
```

## Comparison with similar packages

- 🚀🚀 Blazingly fast - written in rust (see [benchmarks](#Benchmarks))
- Dead simple api modeled after [pymap3d](https://github.com/geospace-code/pymap3d) and [matlab](https://www.mathworks.com/matlabcentral/fileexchange/15285-geodetic-toolbox)
- Exposes rotations (both quaternions and 3x3 matrices)

## Benchmarks

Compared to [pymap3d](https://github.com/geospace-code/pymap3d)

- *~50x* faster for lla2ecef
- *~400x* faster for ecef2lla

<p align="center"><img src="docs/benchmarks.svg"></p>

```bash
# Run benchmarks
uv run pytest --benchmark-histogram="./docs/benchmarks" bench/
```

## Build From Source

Uses standard [maturin](https://github.com/PyO3/maturin) build process

```bash
uv run maturin build -r   # build a whl
uv run maturin dev -r     # build a dev package similar to -e
```
