import copy
import math
import pickle

import numpy as np
import pytest

import rustmap3d

SQUARE = [(-0.05, -0.05), (-0.05, 0.05), (0.05, 0.05), (0.05, -0.05), (-0.05, -0.05)]


class TestConstruction:
    def test_from_vertices(self):
        z = rustmap3d.PolygonZone(SQUARE, 0.0, 10_000.0)
        assert z.vertices == SQUARE
        assert z.alt_min_m == 0.0
        assert z.alt_max_m == 10_000.0
        assert z.alt_buffer_m == 1.0

    def test_buffer_override(self):
        z = rustmap3d.PolygonZone(SQUARE, 0.0, 10_000.0, alt_buffer_m=0.0)
        assert z.alt_buffer_m == 0.0

    def test_unclosed_ring_raises(self):
        with pytest.raises(ValueError, match="closed"):
            rustmap3d.PolygonZone(SQUARE[:-1], 0.0, 10_000.0)

    def test_too_few_vertices_raises(self):
        with pytest.raises(ValueError, match="3"):
            rustmap3d.PolygonZone([(0.0, 0.0), (0.0, 1.0), (0.0, 0.0)], 0.0, 1.0)

    def test_inverted_altitudes_raise(self):
        with pytest.raises(ValueError, match="alt_min_m"):
            rustmap3d.PolygonZone(SQUARE, 1000.0, 0.0)

    def test_negative_buffer_raises(self):
        with pytest.raises(ValueError, match="alt_buffer_m"):
            rustmap3d.PolygonZone(SQUARE, 0.0, 1000.0, alt_buffer_m=-1.0)

    def test_infinite_ceiling(self):
        z = rustmap3d.PolygonZone(SQUARE, 0.0, math.inf)
        assert z.contains((0.0, 0.0, 1e7))

    def test_from_circle_defaults(self):
        z = rustmap3d.PolygonZone.from_circle(36.15, -115.1, 20_000.0, 0.0, 5000.0)
        assert len(z.vertices) == 26
        assert z.vertices[0] == z.vertices[-1]
        assert z.alt_buffer_m == 1.0

    def test_from_circle_overrides(self):
        z = rustmap3d.PolygonZone.from_circle(
            36.15, -115.1, 20_000.0, 0.0, 5000.0, n_sides=8, alt_buffer_m=0.0
        )
        assert len(z.vertices) == 9
        assert z.alt_buffer_m == 0.0

    def test_from_circle_bad_sides_raises(self):
        with pytest.raises(ValueError, match="n_sides"):
            rustmap3d.PolygonZone.from_circle(0.0, 0.0, 1000.0, 0.0, 100.0, n_sides=2)

    def test_from_circle_bad_radius_raises(self):
        with pytest.raises(ValueError, match="radius_m"):
            rustmap3d.PolygonZone.from_circle(0.0, 0.0, -5.0, 0.0, 100.0)

    def test_repr(self):
        r = repr(rustmap3d.PolygonZone(SQUARE, 0.0, 10_000.0))
        assert "PolygonZone" in r
        assert "4" in r


class TestQueries:
    @pytest.fixture
    def square(self):
        return rustmap3d.PolygonZone(SQUARE, 0.0, 10_000.0)

    def test_contains_tuple(self, square):
        assert square.contains((0.0, 0.0, 5000.0))
        assert not square.contains((0.06, 0.0, 5000.0))
        assert not square.contains((0.0, 0.0, 15_000.0))

    def test_contains_geo_position(self, square):
        inside = rustmap3d.GeoPosition.from_lla((0.0, 0.0, 5000.0))
        outside = rustmap3d.GeoPosition.from_lla((0.06, 0.0, 5000.0))
        assert square.contains(inside)
        assert not square.contains(outside)

    def test_altitude_buffer(self):
        with_buffer = rustmap3d.PolygonZone(SQUARE, 0.0, 1000.0)
        without = rustmap3d.PolygonZone(SQUARE, 0.0, 1000.0, alt_buffer_m=0.0)
        assert with_buffer.contains((0.0, 0.0, 1000.5))
        assert not without.contains((0.0, 0.0, 1000.5))

    def test_distance_to(self, square):
        assert square.distance_to((0.0, 0.0, 5000.0)) == 0.0
        # 1 km above the (buffered) ceiling, over the footprint.
        np.testing.assert_allclose(
            square.distance_to((0.0, 0.0, 11_001.0)), 1000.0, atol=1e-6
        )
        lat, lon = rustmap3d.vincenty_direct(0.0, 0.05, 1000.0, 90.0)
        np.testing.assert_allclose(
            square.distance_to((lat, lon, 5000.0)), 1000.0, atol=1.0
        )

    def test_intersects_segment(self, square):
        assert square.intersects_segment((-0.1, 0.0, 5000.0), (0.1, 0.0, 5000.0))
        assert not square.intersects_segment((0.1, -0.1, 5000.0), (0.1, 0.1, 5000.0))
        assert not square.intersects_segment(
            (-0.1, 0.0, 15_000.0), (0.1, 0.0, 15_000.0)
        )
        a = rustmap3d.GeoPosition.from_lla((0.0, 0.0, 15_000.0))
        b = rustmap3d.GeoPosition.from_lla((0.0, 0.0, 5_000.0))
        assert square.intersects_segment(a, b)

    def test_circle_contains_centre_and_not_far_point(self):
        z = rustmap3d.PolygonZone.from_circle(38.0, -77.0, 10_000.0, 0.0, 5000.0)
        assert z.contains((38.0, -77.0, 1000.0))
        assert not z.contains((39.0, -77.0, 1000.0))


class TestPythonProtocol:
    def test_eq(self):
        a = rustmap3d.PolygonZone(SQUARE, 0.0, 10_000.0)
        b = rustmap3d.PolygonZone(SQUARE, 0.0, 10_000.0)
        c = rustmap3d.PolygonZone(SQUARE, 0.0, 9_000.0)
        assert a == b
        assert a != c

    def test_pickle(self):
        a = rustmap3d.PolygonZone(SQUARE, 0.0, 10_000.0, alt_buffer_m=2.0)
        b = pickle.loads(pickle.dumps(a))
        assert a == b
        assert b.alt_buffer_m == 2.0

    def test_deepcopy(self):
        a = rustmap3d.PolygonZone.from_circle(38.0, -77.0, 10_000.0, 0.0, 5000.0)
        b = copy.deepcopy(a)
        assert a == b
        assert b.contains((38.0, -77.0, 1000.0))
