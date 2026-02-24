import numpy as np

import rustmap3d


class TestGeoVelocity:
    def test_construct_from_dir_speed(self):
        actual = rustmap3d.GeoVelocity.from_dir_speed(rustmap3d.DVec3(1, 0, 0), 100.0)
        np.testing.assert_allclose(actual.ecef_uvw.to_tuple(), (100.0, 0, 0))

    def test_dump(self):
        actual = rustmap3d.GeoVelocity.from_dir_speed(rustmap3d.DVec3(1, 0, 0), 100.0).model_dump_json()
        assert actual == '{"dir_ecef":[1.0,0.0,0.0],"speed":100.0}'

    def test_load(self):
        actual = rustmap3d.GeoVelocity.model_validate_json('{"dir_ecef":[1.0,0.0,0.0],"speed":100.0}')
        np.testing.assert_allclose(actual.dir_ecef.x, 1.0)