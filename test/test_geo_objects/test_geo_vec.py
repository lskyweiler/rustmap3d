import numpy as np

import rustmap3d


class TestGeoVector:
    def test_from_ecef(self):
        actual = rustmap3d.GeoVector.from_ecef(
            rustmap3d.DVec3(100.0, 0.0, 0.0), (0, 0, 0)
        )
        np.testing.assert_allclose(actual.ecef_uvw.to_tuple(), (100, 0, 0), atol=1e-10)

    def test_from_ned(self):
        actual = rustmap3d.GeoVector.from_ned(
            rustmap3d.DVec3(100.0, 0.0, 0.0), (0, 0, 0)
        )
        np.testing.assert_allclose(actual.ecef_uvw.to_tuple(), (0, 0, 100), atol=1e-10)

    def test_from_enu(self):
        actual = rustmap3d.GeoVector.from_enu(
            rustmap3d.DVec3(100.0, 0.0, 0.0), (0, 0, 0)
        )
        np.testing.assert_allclose(actual.ecef_uvw.to_tuple(), (0, 100, 0), atol=1e-10)

    def test_from_aer(self):
        actual = rustmap3d.GeoVector.from_aer(
            rustmap3d.DVec3(90.0, 0.0, 100.0), (0, 0, 0)
        )
        np.testing.assert_allclose(actual.ecef_uvw.to_tuple(), (0, 100, 0), atol=1e-10)
