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

    def test_dump_dict(self):
        actual = rustmap3d.GeoVector.from_aer(
            rustmap3d.DVec3(90.0, 0.0, 100.0), (0, 0, 0)
        ).model_dump()
        assert "ecef_uvw" in actual.keys()

    def test_load_dict(self):
        dumped = rustmap3d.GeoVector.from_aer(
            rustmap3d.DVec3(90.0, 0.0, 100.0), (0, 0, 0)
        ).model_dump()
        actual = rustmap3d.GeoVector.model_validate(dumped)
        np.testing.assert_allclose(actual.ecef_uvw.y, 100.0)

    def test_dump(self):
        actual = rustmap3d.GeoVector.from_aer(
            rustmap3d.DVec3(90.0, 0.0, 100.0), (0, 0, 0)
        ).model_dump_json()
        assert "ecef_uvw" in actual
        # assert (
        #     actual
        #     == '{"ecef_uvw":[4.043810359732617e-15,100.00000000000006,0.0],"lla_ref":[0.0,0.0,0.0]}'
        # )

    def test_load(self):
        actual = rustmap3d.GeoVector.model_validate_json(
            '{"ecef_uvw":[4.043810359732617e-15,100.00000000000006,0.0],"lla_ref":[0.0,0.0,0.0]}'
        )
        np.testing.assert_allclose(actual.ecef_uvw.y, 100.0)
