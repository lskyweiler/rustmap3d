"""
Example of the batched API: each *_batch function converts every row of an (N, 3) array
and matches its single-point counterpart bit for bit
"""

import numpy as np

import rustmap3d

# inputs are anything numpy converts to an (N, 3) float64 array
llas = np.array(
    [
        [0.0, 0.0, 100.0],
        [38.9, -77.0, 120.0],
        [-33.9, 151.2, 0.0],
    ]
)

# wgs84 geodetic conversions
ecefs = rustmap3d.lla2ecef_batch(llas)  # > (3, 3) array of x, y, z
llas = rustmap3d.ecef2lla_batch(ecefs)
llas = rustmap3d.ecef2lla_batch([[6378137.0, 0.0, 0.0]])  # lists of rows work too

# local conversions share one reference for the whole batch
lat_ref, lon_ref, alt_ref = 38.9, -77.0, 120.0
enus = rustmap3d.ecef2enu_batch(ecefs, lat_ref, lon_ref, alt_ref_m=alt_ref)
ecefs = rustmap3d.enu2ecef_batch(enus, lat_ref, lon_ref, alt_ref_m=alt_ref)
neds = rustmap3d.lla2ned_batch(llas, lat_ref, lon_ref, alt_ref_m=alt_ref)
llas = rustmap3d.ned2lla_batch(neds, lat_ref, lon_ref, alt_ref_m=alt_ref)
ecef_uvws = rustmap3d.enu2ecef_uvw_batch(enus, lat_ref, lon_ref)
# enu, ned, aer, and their _uvw forms

# aer
aers = rustmap3d.ecef2aer_batch(ecefs, lat_ref, lon_ref, alt_ref_m=alt_ref)
az, el, rng = aers.T  # columns are azimuth, elevation, range
ecefs = rustmap3d.aer2ecef_batch(aers, lat_ref, lon_ref, alt_ref_m=alt_ref)
enus = rustmap3d.aer2enu_batch(aers)
aers = rustmap3d.enu2aer_batch(enus)
# ned2aer, aer2ned, ecef_uvw2aer, aer2ecef_uvw

# anything that isn't shaped (N, 3) raises ValueError
try:
    rustmap3d.lla2ecef_batch([0.0, 0.0, 100.0])
except ValueError:
    ...
