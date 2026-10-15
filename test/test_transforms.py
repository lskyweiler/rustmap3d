"""Checks the Python bindings against the C kinematics-library golden vectors.

The Rust side is validated in depth in crates/map3d/src/transforms/reference_tests.rs; these tests
make sure the bindings pass arguments and units through correctly.
"""

import math

import numpy as np

import rustmap3d


def lla_rad2deg(lat, lon, alt):
    return math.degrees(lat), math.degrees(lon), alt


class TestGoldenVectors:
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
        # C returns azimuth in [-pi, pi]; rustmap3d in [0, 360]
        np.testing.assert_allclose(
            az, math.degrees(-2.61799388) + 360.0, rtol=0, atol=1e-4
        )
        np.testing.assert_allclose(el, math.degrees(-0.6981317), rtol=0, atol=1e-4)
        np.testing.assert_allclose(r, 70.0, rtol=0, atol=1e-6)


class TestEdgeCases:
    def test_ecef2lla_polar_axis(self):
        lat, _, alt = rustmap3d.ecef2lla(0.0, 0.0, 6356752.314245179)
        assert lat == 90.0
        assert abs(alt) < 1e-6
        lat, _, alt = rustmap3d.ecef2lla(0.0, 0.0, -6356752.314245179)
        assert lat == -90.0
        assert abs(alt) < 1e-6
