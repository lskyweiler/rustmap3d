import rustmap3d

import numpy as np


class TestGeoVelocity:
    def test_construct_from_dir_speed(self):
        actual = rustmap3d.GeoVelocity.from_dir_speed(rustmap3d.DVec3(1, 0, 0), 100.0)
        np.testing.assert_allclose(actual.ecef_uvw.to_tuple(), (100.0, 0, 0))
