import rustmap3d


class TestGeoPosition:
    def test_ecef_construct(self):
        actual = rustmap3d.GeoPosition.from_ecef(rustmap3d.dvec3(1e6, 0, 0))
        x = 3
