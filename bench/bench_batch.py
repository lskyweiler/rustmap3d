"""Benchmark batched operations vs vectorized pymap3d equivalents"""

import numpy
import pymap3d

import rustmap3d

rustmap3d.set_global_thread_limits(1)

# --- Batched (N, 3) NumPy arrays vs pymap3d vectorized ---

BATCH_N = 10_000
ecef_batch = numpy.array([rustmap3d.rand_ecef() for _ in range(BATCH_N)])
lla_batch = numpy.array([rustmap3d.rand_lla() for _ in range(BATCH_N)])
LLA_REF = (38.9, -77.0, 120.0)


def test_ecef2lla_batch_rustmap3d(benchmark):
    benchmark(rustmap3d.ecef2lla_batch, ecef_batch)


def test_ecef2geodetic_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.ecef2geodetic, *ecef_batch.T)


def test_lla2ecef_batch_rustmap3d(benchmark):
    benchmark(rustmap3d.lla2ecef_batch, lla_batch)


def test_geodetic2ecef_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.geodetic2ecef, *lla_batch.T)


def test_ecef2aer_batch_rustmap3d(benchmark):
    benchmark(rustmap3d.ecef2aer_batch, ecef_batch, *LLA_REF)


def test_ecef2aer_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.ecef2aer, *ecef_batch.T, *LLA_REF)
