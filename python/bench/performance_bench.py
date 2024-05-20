import pymap3d
import rustmap3d

import numpy as np
from numba import jit

import time


@jit(nopython=True)
def numba_ecef2lla(x: float, y: float, z: float) -> tuple[float, float, float]:
    EARTH_SEMI_MAJOR_AXIS = 6378137.0
    EARTH_SEMI_MAJOR_AXIS_2 = EARTH_SEMI_MAJOR_AXIS**2.0
    EARTH_SEMI_MINOR_AXIS = 6356752.31424518
    EARTH_SEMI_MINOR_AXIS_2 = EARTH_SEMI_MINOR_AXIS**2.0
    EARTH_E = np.sqrt(EARTH_SEMI_MAJOR_AXIS_2 - EARTH_SEMI_MINOR_AXIS_2)
    EARTH_E_2 = EARTH_E**2.0
    EARTH_FLATTENING_FACTOR = 0.003352810664740
    EARTH_ANGULAR_VEL_RADPS = 7.292115900000000e-05

    r = np.sqrt(x**2.0 + y**2.0 + z**2.0)
    u = np.sqrt(
        0.5 * (r**2.0 - EARTH_E_2)
        + 0.5 * np.hypot(r**2.0 - EARTH_E_2, 2.0 * EARTH_E * z)
    )
    hxy = np.hypot(x, y)
    huE = np.hypot(u, EARTH_E)

    Beta = np.pi / 2.0 * np.sign(z)

    if not np.isclose(u, 0.0) and not np.isclose(hxy, 0.0):
        Beta = np.arctan(
            huE / u * z / hxy
        )  # /0 in np-python goes to inf -> crashes in numba
        Beta += (
            (EARTH_SEMI_MINOR_AXIS * u - EARTH_SEMI_MAJOR_AXIS * huE + EARTH_E_2)
            * np.sin(Beta)
        ) / (EARTH_SEMI_MAJOR_AXIS * huE * 1 / np.cos(Beta) - EARTH_E_2 * np.cos(Beta))

    lat = np.arctan(EARTH_SEMI_MAJOR_AXIS / EARTH_SEMI_MINOR_AXIS * np.tan(Beta))
    lon = np.arctan2(y, x)

    alt = np.hypot(
        z - EARTH_SEMI_MINOR_AXIS * np.sin(Beta),
        hxy - EARTH_SEMI_MAJOR_AXIS * np.cos(Beta),
    )
    inside = (
        x**2.0 / EARTH_SEMI_MAJOR_AXIS_2
        + y**2.0 / EARTH_SEMI_MAJOR_AXIS_2
        + z**2.0 / EARTH_SEMI_MINOR_AXIS_2
        < 1
    )
    alt *= -1 if inside else 1
    return lat, lon, alt


def profile_method(ecef_points: list[tuple[float, float, float]], ecef2lla_fn):
    tic = time.time()
    for ecef in ecef_points:
        _ = ecef2lla_fn(*ecef)
    return time.time() - tic


numba_ecef2lla(0, 0, 0)


ecef_points = [rustmap3d.rand_ecef() for _ in range(500000)]
print("profiling rustmap3d")
rustmap3d_prof = profile_method(ecef_points, rustmap3d.ecef2lla)
print("profiling numba")
numba_prof = profile_method(ecef_points, numba_ecef2lla)
print("profiling pymap3d")
pymap3d_prof = profile_method(ecef_points, pymap3d.ecef2geodetic)

print(
    f"rustmap3d: {rustmap3d_prof:0.5f}s | "
    f"numba: {numba_prof:0.5f}s | "
    f"pymap3d: {pymap3d_prof:0.5f}s | "
    f"rust speedup vs pymap3d {pymap3d_prof / rustmap3d_prof}x | "
    f"rust speedup vs numba jit {numba_prof / rustmap3d_prof}x | "
)
