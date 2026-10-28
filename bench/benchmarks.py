"""Benchmark rustmpa3d vs pymap3d both batched and single transormations"""

import numpy
import pytest
import random
import pymap3d

import pymap3d.vincenty
import rustmap3d


BATCH_N = 10_000
ecef_batch = numpy.array([rustmap3d.rand_ecef() for _ in range(BATCH_N)])
lla_batch = numpy.array([rustmap3d.rand_lla() for _ in range(BATCH_N)])
LLA_REF = (38.9, -77.0, 120.0)


def loop_rows(fn_to_test, rows, *args):
    for row in rows:
        fn_to_test(*row, *args)


"""Benchmark batched operations vs python looping over single values"""


@pytest.mark.benchmark(group="rustmap3d-ecef2lla-batch-vs-single")
def test_ecef2lla_loop(benchmark):
    benchmark(loop_rows, rustmap3d.ecef2lla, ecef_batch)


@pytest.mark.benchmark(group="rustmap3d-ecef2lla-batch-vs-single")
def test_ecef2lla_batch(benchmark):
    benchmark(rustmap3d.ecef2lla_batch, ecef_batch)


@pytest.mark.benchmark(group="rustmap3d-lla2ecef-batch-vs-single")
def test_lla2ecef_loop(benchmark):
    benchmark(loop_rows, rustmap3d.lla2ecef, lla_batch)


@pytest.mark.benchmark(group="rustmap3d-lla2ecef-batch-vs-single")
def test_lla2ecef_batch(benchmark):
    benchmark(rustmap3d.lla2ecef_batch, lla_batch)


@pytest.mark.benchmark(group="rustmap3d-ecef2aer-batch-vs-single")
def test_ecef2aer_loop(benchmark):
    benchmark(loop_rows, rustmap3d.ecef2aer, ecef_batch, *LLA_REF)


@pytest.mark.benchmark(group="rustmap3d-ecef2aer-batch-vs-single")
def test_ecef2aer_batch(benchmark):
    benchmark(rustmap3d.ecef2aer_batch, ecef_batch, *LLA_REF)


"""Benchmark batched operations vs vectorized pymap3d equivalents"""


@pytest.mark.benchmark(group="ecef2lla-batch-vs-pymap3d")
def test_ecef2lla_batch_rustmap3d(benchmark):
    benchmark(rustmap3d.ecef2lla_batch, ecef_batch)


@pytest.mark.benchmark(group="ecef2lla-batch-vs-pymap3d")
def test_ecef2geodetic_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.ecef2geodetic, *ecef_batch.T)


@pytest.mark.benchmark(group="lla2ecef-batch-vs-pymap3d")
def test_lla2ecef_batch_rustmap3d(benchmark):
    benchmark(rustmap3d.lla2ecef_batch, lla_batch)


@pytest.mark.benchmark(group="lla2ecef-batch-vs-pymap3d")
def test_geodetic2ecef_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.geodetic2ecef, *lla_batch.T)


@pytest.mark.benchmark(group="ecef2aer-batch-vs-pymap3d")
def test_ecef2aer_batch_rustmap3d(benchmark):
    benchmark(rustmap3d.ecef2aer_batch, ecef_batch, *LLA_REF)


@pytest.mark.benchmark(group="ecef2aer-batch-vs-pymap3d")
def test_ecef2aer_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.ecef2aer, *ecef_batch.T, *LLA_REF)


"""Benchmark batch functions using parallel threads vs serial execution"""


@pytest.mark.benchmark(group="rustmap3d-ecef2lla-batch-par-vs-serial")
def test_ecef2lla_batch_serial(benchmark):
    with rustmap3d.threadpool_limits(1):
        benchmark(rustmap3d.ecef2lla_batch, ecef_batch)


@pytest.mark.benchmark(group="rustmap3d-ecef2lla-batch-par-vs-serial")
def test_ecef2lla_batch_parallel(benchmark):
    benchmark(rustmap3d.ecef2lla_batch, ecef_batch)


@pytest.mark.benchmark(group="rustmap3d-lla2ecef-batch-par-vs-serial")
def test_lla2ecef_batch_serial(benchmark):
    with rustmap3d.threadpool_limits(1):
        benchmark(rustmap3d.lla2ecef_batch, lla_batch)


@pytest.mark.benchmark(group="rustmap3d-lla2ecef-batch-par-vs-serial")
def test_lla2ecef_batch_parallel(benchmark):
    benchmark(rustmap3d.lla2ecef_batch, lla_batch)


@pytest.mark.benchmark(group="rustmap3d-ecef2aer-batch-par-vs-serial")
def test_ecef2aer_batch_serial(benchmark):
    with rustmap3d.threadpool_limits(1):
        benchmark(rustmap3d.ecef2aer_batch, ecef_batch, *LLA_REF)


@pytest.mark.benchmark(group="rustmap3d-ecef2aer-batch-par-vs-serial")
def test_ecef2aer_batch_parallel(benchmark):
    benchmark(rustmap3d.ecef2aer_batch, ecef_batch, *LLA_REF)


"""Single point transformation benches"""

ITERATIONS = 1000
ROUNDS = 10
N = 10
ecef_points = [rustmap3d.rand_ecef() for _ in range(N)]
lla_points = [rustmap3d.rand_lla() for _ in range(N)]


def iter_points(fn_to_test, points):
    for p in points:
        fn_to_test(*p)


@pytest.mark.benchmark(group="single-ecef2lla")
def test_ecef2lla_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.ecef2lla, ecef_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-ecef2lla")
def test_ecef2geodetic_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.ecef2geodetic, ecef_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-lla2ecef")
def test_lla2ecef_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.lla2ecef, lla_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-lla2ecef")
def test_geodetic2ecef_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.geodetic2ecef, lla_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


"""Benchmark vincenty functions for single points vs pymap3d"""


rng = random.Random(0)

# Direct: (lat_deg, lon_deg, range_m, bearing_deg). Latitudes stay away from the poles.
direct_points = [
    (
        rng.uniform(-80, 80),
        rng.uniform(-180, 180),
        rng.uniform(1e3, 5e6),
        rng.uniform(0, 360),
    )
    for _ in range(N)
]

# Inverse: (lat_a_deg, lon_a_deg, lat_b_deg, lon_b_deg). Points are generated from the
# direct set so they stay away from the antipodal region where Vincenty fails to converge.
inverse_points = [
    (lat, lon, *rustmap3d.vincenty_direct(lat, lon, rng_m, brg))
    for lat, lon, rng_m, brg in direct_points
]


@pytest.mark.benchmark(group="single-vincenty-direct")
def test_vincenty_direct_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.vincenty_direct, direct_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-vincenty-direct")
def test_vreckon_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.vincenty.vreckon, direct_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-vincenty-inverse")
def test_vincenty_inverse_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.vincenty_inverse, inverse_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-vincenty-inverse")
def test_vdist_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.vincenty.vdist, inverse_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )
