import numpy as np

import rustmap3d

# todo: these are thoroughly tested in rust, but need to test python side


class TestECEF2LLA:
    def test_lla2ecef_zero(self):
        actual = rustmap3d.lla2ecef(0.0, 0.0, 0.0)
        np.testing.assert_allclose(actual, [6378137.0, 0, 0], 1e-4)
