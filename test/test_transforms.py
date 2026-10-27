"""Checks the Python bindings against published test vectors.

The Rust side is validated in depth in crates/map3d/src/transforms/reference_tests/; these tests
make sure the bindings pass arguments and units through correctly.
"""

import math

import numpy as np

import rustmap3d


def lla_rad2deg(lat, lon, alt):
    return math.degrees(lat), math.degrees(lon), alt


class TestVectors:
    def test_ecef2lla(self):
        actual = rustmap3d.ecef2lla(4040000.0, 4813000.0, 1100000.0)
        expected = lla_rad2deg(0.174440593201424, 0.872492313234789, 1888.1851099917756)
        np.testing.assert_allclose(
            actual[:2], expected[:2], rtol=0, atol=math.degrees(1e-12)
        )
        np.testing.assert_allclose(actual[2], expected[2], rtol=0, atol=1e-4)

    def test_lla2ecef(self):
        actual = rustmap3d.lla2ecef(*lla_rad2deg(-0.8, 1.7, 4750.0))
        expected = (-573960.23482409, 4417543.57131158, -4556021.90733023)
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-6)

    def test_ned2aer(self):
        az, el, r = rustmap3d.ned2aer(-46.43897628, -26.81155573, 44.99513264)
        # the test vector gives azimuth in [-pi, pi]; rustmap3d in [0, 360]
        np.testing.assert_allclose(
            az, math.degrees(-2.61799388) + 360.0, rtol=0, atol=1e-4
        )
        np.testing.assert_allclose(el, math.degrees(-0.6981317), rtol=0, atol=1e-4)
        np.testing.assert_allclose(r, 70.0, rtol=0, atol=1e-6)

    def test_aer2ned(self):
        actual = rustmap3d.aer2ned(30.0, 40.0, -70.0)
        expected = (-46.43897628, -26.81155573, 44.99513264)
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-6)

    def test_ecef2ned_with_ref_altitude(self):
        lat_ref, lon_ref, alt_ref = lla_rad2deg(2.1, -1.1, 1250.0)
        actual = rustmap3d.ecef2ned(
            -1465000.0, 2880000.0, 5483500.0, lat_ref, lon_ref, alt_ref_m=alt_ref
        )
        expected = (2220.90854391709, 738.047215659648, -1205.58533167388)
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-6)

    def test_lla2ned_with_ref_altitude(self):
        lla = lla_rad2deg(-0.707, 0.496, 2400.0)
        lat_ref, lon_ref, alt_ref = lla_rad2deg(-0.7, 0.5, 6000.0)
        actual = rustmap3d.lla2ned(*lla, lat_ref, lon_ref, alt_ref_m=alt_ref)
        expected = (-44576.5574572138, -19432.24609638221, 3785.6580070712043)
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-6)

    def test_ned2ecef_with_ref_altitude(self):
        lat_ref, lon_ref, alt_ref = lla_rad2deg(-2.9, -0.5, 6000.0)
        actual = rustmap3d.ned2ecef(
            -6500.0, -4800.0, 2400.0, lat_ref, lon_ref, alt_ref_m=alt_ref
        )
        expected = (-5442559.838214509, 2967814.4198892456, -1510590.1708055406)
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-6)


class TestEdgeCases:
    def test_ecef2lla_polar_axis(self):
        lat, _, alt = rustmap3d.ecef2lla(0.0, 0.0, 6356752.314245179)
        assert lat == 90.0
        assert abs(alt) < 1e-6
        lat, _, alt = rustmap3d.ecef2lla(0.0, 0.0, -6356752.314245179)
        assert lat == -90.0
        assert abs(alt) < 1e-6

    def test_dd2dms_carries_rounded_seconds(self):
        assert rustmap3d.dd2dms(10.0166666, True) == "10:1:0.000N"
        assert rustmap3d.dd2dms(45.9999999, True) == "46:0:0.000N"

    def test_ll2dms_uses_longitude(self):
        assert rustmap3d.ll2dms(25.379094, -74.998757) == (
            "25:22:44.738N",
            "74:59:55.525W",
        )
