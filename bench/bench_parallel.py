"""Benchmark batch functions using parallel threads vs serial execution"""

import numpy
import rustmap3d

# --- Batched (N, 3) NumPy arrays vs pymap3d vectorized ---

BATCH_N = 10_000
ecef_batch = numpy.array([rustmap3d.rand_ecef() for _ in range(BATCH_N)])
lla_batch = numpy.array([rustmap3d.rand_lla() for _ in range(BATCH_N)])
LLA_REF = (38.9, -77.0, 120.0)


def test_ecef2lla_batch_serial(benchmark):
    with rustmap3d.threadpool_limits(1):
        benchmark(rustmap3d.ecef2lla_batch, ecef_batch)


def test_lla2ecef_batch_serial(benchmark):
    with rustmap3d.threadpool_limits(1):
        benchmark(rustmap3d.lla2ecef_batch, lla_batch)


def test_ecef2aer_batch_serial(benchmark):
    with rustmap3d.threadpool_limits(1):
        benchmark(rustmap3d.ecef2aer_batch, ecef_batch, *LLA_REF)


def test_ecef2lla_batch_parallel(benchmark):
    benchmark(rustmap3d.ecef2lla_batch, ecef_batch)


def test_lla2ecef_batch_parallel(benchmark):
    benchmark(rustmap3d.lla2ecef_batch, lla_batch)


def test_ecef2aer_batch_parallel(benchmark):
    benchmark(rustmap3d.ecef2aer_batch, ecef_batch, *LLA_REF)
