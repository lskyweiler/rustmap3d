import numpy as np

import rustmap3d

# todo: these are thoroughly tested in rust, but need to test python side


class TestECEF2LLA:
    def test_lla2ecef_zero(self):
        actual = rustmap3d.lla2ecef(0.0, 0.0, 0.0)
        np.testing.assert_allclose(actual, [6378137.0, 0, 0], 1e-4)


class TestDegreesMinutesSeconds:
    def test_dms2dd(self):
        actual = rustmap3d.dms2dd("25:22:44.738N")
        np.testing.assert_allclose(actual, 25.37909389)

    def test_dd2dms_lat(self):
        actual = rustmap3d.dd2dms(25.37909389, is_lat=True)
        assert actual == "25:22:44.738N"

    def test_dd2dms_lon(self):
        actual = rustmap3d.dd2dms(25.37909389, is_lat=False)
        assert actual == "25:22:44.738E"
