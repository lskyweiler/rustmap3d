"""
Example of the low-level, simple API modelled after pymap3d and matlab
"""

import rustmap3d


# wgs84 geodetic conversions
x, y, z = rustmap3d.lla2ecef(0.0, 0.0, 100.0)
lat, lon, alt = rustmap3d.ecef2lla(x, y, z)
...

# local conversions
ecef_uvw = rustmap3d.enu2ecef_uvw(100.0, 0.0, 0.0, lat, lon)
ecef = rustmap3d.enu2ecef(100.0, 0.0, 0.0, lat, lon)
ecef_uvw = rustmap3d.ned2ecef_uvw(0.0, 100.0, 0.0, lat, lon)
ecef = rustmap3d.ned2ecef(0.0, 100.0, 0.0, lat, lon)
# enu, ned, aer
...

# local rotations
enu_quat = rustmap3d.enu2ecef_quat(lat, lon)
enu_dcm = rustmap3d.enu2ecef_dcm(lat, lon)
# enu, ned

# Conversions
dd = rustmap3d.dms2dd("25:22:44.738N")  # > 25.37909389
dms = rustmap3d.dd2dms(25.37909389, is_lat=True)  # > "25:22:44.738N"
lat, lon = rustmap3d.ll2dms(
    25.37909389, -138.7895679
)  # > "25:22:44.738N", "138:47:22.444W"
...

# distance functions
lat, lon = rustmap3d.vincenty_direct(
    lat_d=0.0, lon_d=0.0, range_m=100.0, bearing_d=10.0
)
range_m, bearing_ab, bearing_ba = rustmap3d.vincenty_inverse(0.0, 0.0, 10.0, 10.0)
