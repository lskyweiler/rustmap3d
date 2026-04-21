import copy
import pickle

import numpy as np
import pydantic
import rustmap3d


class TestGeoVelocity:
    def test_construct_from_dir_speed(self):
        actual = rustmap3d.GeoVelocity.from_dir_speed(rustmap3d.DVec3(1, 0, 0), 100.0)
        np.testing.assert_allclose(actual.ecef_uvw.to_tuple(), (100.0, 0, 0))

    def test_dump_dict(self):
        actual = rustmap3d.GeoVelocity.from_dir_speed(
            rustmap3d.DVec3(1, 0, 0), 100.0
        ).model_dump()
        assert "dir_ecef" in actual.keys()

    def test_load_dict(self):
        dumped = rustmap3d.GeoVelocity.from_dir_speed(
            rustmap3d.DVec3(1, 0, 0), 100.0
        ).model_dump()
        rustmap3d.GeoVelocity.model_validate(dumped)

    def test_dump(self):
        actual = rustmap3d.GeoVelocity.from_dir_speed(
            rustmap3d.DVec3(1, 0, 0), 100.0
        ).model_dump_json()
        assert actual == '{"dir_ecef":[1.0,0.0,0.0],"speed":100.0}'

    def test_load(self):
        actual = rustmap3d.GeoVelocity.model_validate_json(
            '{"dir_ecef":[1.0,0.0,0.0],"speed":100.0}'
        )
        np.testing.assert_allclose(actual.dir_ecef.x, 1.0)


class TestDeepcopy:
    def test_deepcopy(self):
        original = rustmap3d.GeoVelocity.from_dir_speed(
            rustmap3d.DVec3(0.707, 0.707, 0.0), 100.0
        )
        copied = copy.deepcopy(original)

        # Verify values match
        np.testing.assert_allclose(
            copied.dir_ecef.to_tuple(), original.dir_ecef.to_tuple()
        )
        np.testing.assert_allclose(copied.speed, original.speed)

        # Verify it's a different object
        assert copied is not original

    def test_pickle(self):
        original = rustmap3d.GeoVelocity.from_dir_speed(
            rustmap3d.DVec3(0.707, 0.707, 0.0), 100.0
        )
        pickled = pickle.dumps(original)
        unpickled = pickle.loads(pickled)

        # Verify values match
        np.testing.assert_allclose(
            unpickled.dir_ecef.to_tuple(), original.dir_ecef.to_tuple()
        )
        np.testing.assert_allclose(unpickled.speed, original.speed)


class MockModel(pydantic.BaseModel):
    vel: rustmap3d.GeoVelocity


class TestPydantic:
    def test_model_dump(self):
        model = MockModel(
            vel=rustmap3d.GeoVelocity.from_dir_speed(rustmap3d.DVec3(1, 0, 0), 100.0)
        )
        dumped = model.model_dump(mode="json")
        assert "vel" in dumped
        assert "dir_ecef" in dumped["vel"]

    def test_model_dump_json(self):
        model = MockModel(
            vel=rustmap3d.GeoVelocity.from_dir_speed(rustmap3d.DVec3(1, 0, 0), 100.0)
        )
        dumped = model.model_dump_json()
        assert isinstance(dumped, str)
        assert "vel" in dumped
        assert "dir_ecef" in dumped

    def test_model_validate(self):
        model = MockModel(
            vel=rustmap3d.GeoVelocity.from_dir_speed(rustmap3d.DVec3(1, 0, 0), 100.0)
        )
        dumped = model.model_dump(mode="json")
        actual = MockModel.model_validate(dumped)
        np.testing.assert_allclose(actual.vel.dir_ecef.to_tuple(), (1, 0, 0))
        np.testing.assert_allclose(actual.vel.speed, 100.0)

    def test_model_validate_json(self):
        model = MockModel(
            vel=rustmap3d.GeoVelocity.from_dir_speed(rustmap3d.DVec3(1, 0, 0), 100.0)
        )
        dumped = model.model_dump_json()
        actual = MockModel.model_validate_json(dumped)
        np.testing.assert_allclose(actual.vel.dir_ecef.to_tuple(), (1, 0, 0))
        np.testing.assert_allclose(actual.vel.speed, 100.0)

    def test_model_json_schema(self):
        actual = MockModel.model_json_schema()
        assert "DVec3" in actual["$defs"]
