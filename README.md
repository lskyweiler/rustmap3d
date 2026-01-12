# rustmap3d

[![License](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/lskyweiler/rustmap3d#license)
![Python](https://img.shields.io/badge/Python_3.8_|_3.9_|_3.10_|_3.11_|_3.12_|_3.13-blue?logo=python&logoColor=fff)
[![PyPI](https://img.shields.io/badge/PyPI-3775A9?logo=pypi&logoColor=fff)](https://pypi.org/project/rustmap3d/)
[![crates.io](https://img.shields.io/crates/v/map3d.svg)](https://crates.io/crates/map3d)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)

<p align="center"><img src="docs/rustmap3d.logo.svg" width="300px" height="300px"/></p>

**Simple, fast, and ergonomic geodetic coordinate conversions**

---
### Installation
```bash
# add to an existing project
uv add rustmap3d
# pip install it directly
uv pip install rustmap3d

# pip install without uv
pip install rustmap3d
```
---
### Examples
```python
import rustmap3d


# wgs84 geodetic conversions
lla = rustmap3d.ecef2lla(x, y, z)
ecef = rustmap3d.lla2ecef(lat, lon, alt)
...

# local conversions
ecef_uvw = rustmap3d.enu2ecef_uvw(e, n, u, lat_ref, lon_ref)
ecef = rustmap3d.enu2ecef(e, n, u, lat_ref, lon_ref)
ecef_uvw = rustmap3d.ned2ecef_uvw(n, e, d, lat_ref, lon_ref)
ecef = rustmap3d.ned2ecef(n, e, d, lat_ref, lon_ref)
# enu, ned, aer
...  

# local rotations
enu_quat = rustmap3d.enu2ecef_quat(lat, lon)
enu_dcm = rustmap3d.enu2ecef_dcm(lat, lon) 
# enu, ned

# Conversions
dd = rustmap3d.dms2dd("25:22:44.738N")  #> 25.37909389
dms = rustmap3d.dd2dms(25.37909389, is_lat=true) #> "25:22:44.738N"
lat, lon = rustmap3d.ll2dms(25.37909389, -138.7895679)  #> "25:22:44.738N", "138:47:22.444W"
...  

# distance functions
lat, lon = rustmap3d.vincenty_direct(lat_deg, lon_deg, range_m, bearing_deg)
range_m, bearing_ab, bearing_ba = rustmap3d.vincenty_inverse(lat_a, lon_a, lat_b, lon_b)
```

## Comparison with similar packages

- 🚀🚀 Blazingly fast - written in rust (see [benchmarks](#benchmarks))
- Zero dependencies
- Dead simple api modeled after [pymap3d](https://github.com/geospace-code/pymap3d) and [matlab](https://www.mathworks.com/matlabcentral/fileexchange/15285-geodetic-toolbox)
- Exposes rotations (both quaternions and 3x3 matrices)

## Benchmarks

Compared to [pymap3d](https://github.com/geospace-code/pymap3d)
> Note: This is comparing single calls of python floats, not vectorized numpy performance. Vectorized batched conversions will be implemented in the future

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
