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
