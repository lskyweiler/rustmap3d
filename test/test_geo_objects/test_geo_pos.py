import numpy as np

import rustmap3d


class TestGeoPosition:
    def test_ecef_construct(self):
        actual = rustmap3d.GeoPosition.from_ecef(rustmap3d.DVec3(1e6, 0, 0))
        np.testing.assert_allclose(actual.ecef.to_tuple(), (1e6, 0, 0))

    def test_lla_construct(self):
        actual = rustmap3d.GeoPosition.from_lla((0, 0, 0))
        np.testing.assert_allclose(actual.lla, (0.0, 0.0, 0.0), atol=1e-5)

    def test_distance(self):
        a = rustmap3d.GeoPosition.from_lla((0, 0, 0))
        b = rustmap3d.GeoPosition.from_ned(rustmap3d.DVec3(100, 0, 0), (0, 0, 0))
        actual = a.distance(b)
        np.testing.assert_allclose(actual, 100.0, atol=1e-5)

    def test_lla_dms(self):
        actual = rustmap3d.GeoPosition.from_lla((0, 0, 0)).lat_lon_dms()
        assert actual == "0:0:0.000S, 0:0:0.000W"


class TestGeoPosOps:
    def test_sub(self):
        a = rustmap3d.GeoPosition.from_lla((0, 0, 0))
        b = rustmap3d.GeoPosition.from_ned(rustmap3d.DVec3(100, 0, 0), (0, 0, 0))
        actual = b - a
        np.testing.assert_allclose(actual.ecef_uvw.to_tuple(), (0, 0, 100))

    def test_rsub(self):
        a = rustmap3d.GeoPosition.from_lla((0, 0, 0))
        b = rustmap3d.GeoPosition.from_ned(rustmap3d.DVec3(100, 0, 0), (0, 0, 0))
        actual = b - a
        np.testing.assert_allclose(actual.ecef_uvw.to_tuple(), (0, 0, 100))

    def test_add(self):
        a = rustmap3d.GeoPosition.from_lla((0, 0, 0))
        b = rustmap3d.GeoVector.from_ned(rustmap3d.DVec3(100, 0, 0), (0, 0, 0))
        actual = a + b
        np.testing.assert_allclose(actual.ecef.z, 100.0)

    def test_radd(self):
        a = rustmap3d.GeoPosition.from_lla((0, 0, 0))
        b = rustmap3d.GeoVector.from_ned(rustmap3d.DVec3(100, 0, 0), (0, 0, 0))
        actual = b + a
        np.testing.assert_allclose(actual.ecef.z, 100.0)

    def test_rate_times_time(self):
        rate = rustmap3d.GeoVelocity.from_dir_speed(
            rustmap3d.DVec3(0.0, 1.0, 0.0), 100.0
        )
        pos = rustmap3d.GeoPosition.from_lla((0, 0, 0))

        actual = pos + rate * 1.0
        np.testing.assert_allclose(actual.ecef.y, 100.0)
