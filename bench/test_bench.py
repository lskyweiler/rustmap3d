import numpy
import pymap3d

import rustmap3d

ITERATIONS = 1000
ROUNDS = 10
N = 10
ecef_points = [rustmap3d.rand_ecef() for _ in range(N)]
lla_points = [rustmap3d.rand_lla() for _ in range(N)]


def iter_points(fn_to_test, points):
    for p in points:
        fn_to_test(*p)


def test_ecef2lla_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.ecef2lla, ecef_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


def test_ecef2geodetic_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.ecef2geodetic, ecef_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


def test_lla2ecef_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.lla2ecef, lla_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


def test_geodetic2ecef_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.geodetic2ecef, lla_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


# --- Batched (N, 3) NumPy arrays vs. looping over the single-point functions ---

BATCH_N = 10_000
ecef_batch = numpy.array([rustmap3d.rand_ecef() for _ in range(BATCH_N)])
lla_batch = numpy.array([rustmap3d.rand_lla() for _ in range(BATCH_N)])
LLA_REF = (38.9, -77.0, 120.0)


def loop_rows(fn_to_test, rows, *args):
    for row in rows:
        fn_to_test(*row, *args)


def test_ecef2lla_loop_rustmap3d(benchmark):
    benchmark(loop_rows, rustmap3d.ecef2lla, ecef_batch)


def test_ecef2lla_batch_rustmap3d(benchmark):
    benchmark(rustmap3d.ecef2lla_batch, ecef_batch)


def test_ecef2geodetic_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.ecef2geodetic, *ecef_batch.T)


def test_lla2ecef_loop_rustmap3d(benchmark):
    benchmark(loop_rows, rustmap3d.lla2ecef, lla_batch)


def test_lla2ecef_batch_rustmap3d(benchmark):
    benchmark(rustmap3d.lla2ecef_batch, lla_batch)


def test_geodetic2ecef_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.geodetic2ecef, *lla_batch.T)


def test_ecef2aer_loop_rustmap3d(benchmark):
    benchmark(loop_rows, rustmap3d.ecef2aer, ecef_batch, *LLA_REF)


def test_ecef2aer_batch_rustmap3d(benchmark):
    benchmark(rustmap3d.ecef2aer_batch, ecef_batch, *LLA_REF)


def test_ecef2aer_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.ecef2aer, *ecef_batch.T, *LLA_REF)
