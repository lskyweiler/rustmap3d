import numpy as np
import rustmap3d


class TestVincentyDirect:
    def test_direct(self):
        actual = rustmap3d.vincenty_direct(45.0, 120.0, 0.0, 0.0)
        np.testing.assert_allclose(actual, (45.0, 120.0))


class TestVincentyInverse:
    def test_direct(self):
        actual = rustmap3d.vincenty_inverse(45.0, 120.0, 45.0, 120.0)
        np.testing.assert_allclose(actual, (0.0, 0.0, 0.0))
