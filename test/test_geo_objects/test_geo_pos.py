import rustmap3d
import numpy as np


class TestGeoPosition:
    def test_ecef_construct(self):
        actual = rustmap3d.GeoPosition.from_ecef(rustmap3d.DVec3(1e6, 0, 0))
        assert actual.ecef.x == 1e6
        assert actual.ecef.y == 0
        assert actual.ecef.z == 0

    def test_lla_construct(self):
        actual = rustmap3d.GeoPosition.from_lla((0, 0, 0))
        np.testing.assert_allclose(actual.lla, (0.0, 0.0, 0.0), atol=1e-5)
