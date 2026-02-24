import math

import numpy as np

import rustmap3d


class TestGeoOrientation:
    def test_from_axis(self):
        actual = rustmap3d.GeoOrientation.from_axis_angle(
            rustmap3d.DVec3(0, 0, 1), math.pi / 2
        )
        np.testing.assert_allclose(actual.x_axis().to_tuple(), (0, 1, 0), atol=1e-10)

    def test_to_euler(self):
        rot = rustmap3d.GeoOrientation.from_axis_angle(
            rustmap3d.DVec3(0, 0, 1), math.pi / 2
        )
        rpy = rot.to_ecef_euler(rustmap3d.EulerRot.XYZ)
        np.testing.assert_allclose(rpy, (0, 0, math.pi / 2.0), atol=1e-10)

    def test_from_ecef_euler(self):
        actual = rustmap3d.GeoOrientation.from_ecef_euler(
            rustmap3d.DVec3(0, 0, math.pi / 2.0)
        )
        np.testing.assert_allclose(actual.x_axis().to_tuple(), (0, 1, 0), atol=1e-10)

    def test_dump_dict(self):
        actual = rustmap3d.GeoOrientation.from_ecef_euler(
            rustmap3d.DVec3(0, 0, math.pi / 2.0)
        ).model_dump()
        assert "ecef_rot" in actual.keys()

    def test_load_dict(self):
        dumped = rustmap3d.GeoOrientation.from_ecef_euler(
            rustmap3d.DVec3(0, 0, math.pi / 2.0)
        ).model_dump()
        rustmap3d.GeoOrientation.model_validate(dumped)

    def test_dump(self):
        actual = rustmap3d.GeoOrientation.from_ecef_euler(
            rustmap3d.DVec3(0, 0, math.pi / 2.0)
        ).model_dump_json()
        assert actual == '{"ecef_rot":[0.0,0.0,0.7071067811865476,0.7071067811865476]}'

    def test_load(self):
        rustmap3d.GeoOrientation.model_validate_json(
            '{"ecef_rot":[0.0,0.0,0.7071067811865476,0.7071067811865476]}'
        )
