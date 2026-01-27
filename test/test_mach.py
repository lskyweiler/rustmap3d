import numpy as np
import pytest
import rustmap3d


class TestMach:
    def test_mach_ground(self):
        actual = rustmap3d.mach(343, 0.0)
        np.testing.assert_allclose(actual, 1.0079341757272995)

    def test_mach_neg(self):
        with pytest.raises(ValueError):
            rustmap3d.mach(343, -10.)

    def test_mach_larger(self):
        with pytest.raises(ValueError):
            rustmap3d.mach(343, 100e10)
