"""Benchmark batched operations vs python looping over single values"""

import numpy

import rustmap3d

# --- Batched (N, 3) NumPy arrays vs. looping over the single-point functions ---

BATCH_N = 10_000
ecef_batch = numpy.array([rustmap3d.rand_ecef() for _ in range(BATCH_N)])
lla_batch = numpy.array([rustmap3d.rand_lla() for _ in range(BATCH_N)])
LLA_REF = (38.9, -77.0, 120.0)


def loop_rows(fn_to_test, rows, *args):
    for row in rows:
        fn_to_test(*row, *args)


def test_ecef2lla_loop(benchmark):
    benchmark(loop_rows, rustmap3d.ecef2lla, ecef_batch)


def test_ecef2lla_batch(benchmark):
    benchmark(rustmap3d.ecef2lla_batch, ecef_batch)


def test_lla2ecef_loop(benchmark):
    benchmark(loop_rows, rustmap3d.lla2ecef, lla_batch)


def test_lla2ecef_batch(benchmark):
    benchmark(rustmap3d.lla2ecef_batch, lla_batch)


def test_ecef2aer_loop(benchmark):
    benchmark(loop_rows, rustmap3d.ecef2aer, ecef_batch, *LLA_REF)


def test_ecef2aer_batch(benchmark):
    benchmark(rustmap3d.ecef2aer_batch, ecef_batch, *LLA_REF)
