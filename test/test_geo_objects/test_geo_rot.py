import copy
import math
import pickle

import numpy as np
import pydantic

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
        # assert actual == '{"ecef_rot":[0.0,0.0,0.7071067811865476,0.7071067811865476]}
        assert "ecef_rot" in actual

    def test_load(self):
        rustmap3d.GeoOrientation.model_validate_json(
            '{"ecef_rot":[0.0,0.0,0.7071067811865476,0.7071067811865476]}'
        )


class TestDeepcopy:
    def test_deepcopy(self):
        original = rustmap3d.GeoOrientation.from_ecef_euler(
            rustmap3d.DVec3(0.1, 0.2, 0.3)
        )
        copied = copy.deepcopy(original)

        # Verify values match by checking the resulting rotation produces the same x_axis
        np.testing.assert_allclose(
            copied.x_axis().to_tuple(), original.x_axis().to_tuple(), atol=1e-10
        )
        np.testing.assert_allclose(
            copied.y_axis().to_tuple(), original.y_axis().to_tuple(), atol=1e-10
        )
        np.testing.assert_allclose(
            copied.z_axis().to_tuple(), original.z_axis().to_tuple(), atol=1e-10
        )

        # Verify it's a different object
        assert copied is not original

    def test_pickle(self):
        original = rustmap3d.GeoOrientation.from_ecef_euler(
            rustmap3d.DVec3(0.1, 0.2, 0.3)
        )
        pickled = pickle.dumps(original)
        unpickled = pickle.loads(pickled)

        # Verify values match
        np.testing.assert_allclose(
            unpickled.x_axis().to_tuple(), original.x_axis().to_tuple(), atol=1e-10
        )
        np.testing.assert_allclose(
            unpickled.y_axis().to_tuple(), original.y_axis().to_tuple(), atol=1e-10
        )
        np.testing.assert_allclose(
            unpickled.z_axis().to_tuple(), original.z_axis().to_tuple(), atol=1e-10
        )


class MockModel(pydantic.BaseModel):
    rot: rustmap3d.GeoOrientation


class TestPydantic:
    def test_model_dump(self):
        model = MockModel(
            rot=rustmap3d.GeoOrientation.from_ecef_euler(
                rustmap3d.DVec3(0, 0, math.pi / 2.0)
            )
        )
        dumped = model.model_dump(mode="json")
        assert "rot" in dumped
        assert "ecef_rot" in dumped["rot"]

    def test_model_dump_json(self):
        model = MockModel(
            rot=rustmap3d.GeoOrientation.from_ecef_euler(
                rustmap3d.DVec3(0, 0, math.pi / 2.0)
            )
        )
        dumped = model.model_dump_json()
        assert isinstance(dumped, str)
        assert "rot" in dumped
        assert "ecef_rot" in dumped

    def test_model_validate(self):
        model = MockModel(
            rot=rustmap3d.GeoOrientation.from_ecef_euler(
                rustmap3d.DVec3(0, 0, math.pi / 2.0)
            )
        )
        dumped = model.model_dump(mode="json")
        _ = MockModel.model_validate(dumped)

    def test_model_validate_json(self):
        model = MockModel(
            rot=rustmap3d.GeoOrientation.from_ecef_euler(
                rustmap3d.DVec3(0, 0, math.pi / 2.0)
            )
        )
        dumped = model.model_dump_json()
        _ = MockModel.model_validate_json(dumped)

    def test_model_json_schema(self):
        actual = MockModel.model_json_schema()
        assert "DQuat" in actual["$defs"]
