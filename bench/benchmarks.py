"""Benchmark rustmpa3d vs pymap3d both batched and single transormations"""

import random

import numpy
import pymap3d
import pymap3d.vincenty
import pytest

import rustmap3d

BATCH_N = 10_000
ECEF_BATCH = numpy.array([rustmap3d.rand_ecef() for _ in range(BATCH_N)])
LLA_BATCH = numpy.array([rustmap3d.rand_lla() for _ in range(BATCH_N)])
LLA_REF = (38.9, -77.0, 120.0)


def loop_rows(fn_to_test, rows, *args):
    for row in rows:
        fn_to_test(*row, *args)


"""Single point transformation benches"""

ITERATIONS = 1000
ROUNDS = 10
N = 10
ECEF_POINTS = [rustmap3d.rand_ecef() for _ in range(N)]
LLA_POINTS = [rustmap3d.rand_lla() for _ in range(N)]


def iter_points(fn_to_test, points):
    for p in points:
        fn_to_test(*p)


@pytest.mark.benchmark(group="single-point-ecef2lla")
def test_ecef2lla_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.ecef2lla, ECEF_POINTS),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-point-ecef2lla")
def test_ecef2geodetic_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.ecef2geodetic, ECEF_POINTS),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-point-lla2ecef")
def test_lla2ecef_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.lla2ecef, LLA_POINTS),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-point-lla2ecef")
def test_geodetic2ecef_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.geodetic2ecef, LLA_POINTS),
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


@pytest.mark.benchmark(group="single-point-vincenty-direct")
def test_vincenty_direct_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.vincenty_direct, direct_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-point-vincenty-direct")
def test_vreckon_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.vincenty.vreckon, direct_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-point-vincenty-inverse")
def test_vincenty_inverse_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.vincenty_inverse, inverse_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


@pytest.mark.benchmark(group="single-point-vincenty-inverse")
def test_vdist_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.vincenty.vdist, inverse_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


"""Benchmark batched operations vs python looping over single values"""


@pytest.mark.benchmark(group="rustmap3d-ecef2lla-batch-vs-single")
def test_ecef2lla_loop(benchmark):
    benchmark(loop_rows, rustmap3d.ecef2lla, ECEF_BATCH)


@pytest.mark.benchmark(group="rustmap3d-ecef2lla-batch-vs-single")
def test_ecef2lla_batch(benchmark):
    benchmark(rustmap3d.ecef2lla_batch, ECEF_BATCH)


@pytest.mark.benchmark(group="rustmap3d-lla2ecef-batch-vs-single")
def test_lla2ecef_loop(benchmark):
    benchmark(loop_rows, rustmap3d.lla2ecef, LLA_BATCH)


@pytest.mark.benchmark(group="rustmap3d-lla2ecef-batch-vs-single")
def test_lla2ecef_batch(benchmark):
    benchmark(rustmap3d.lla2ecef_batch, LLA_BATCH)


@pytest.mark.benchmark(group="rustmap3d-ecef2aer-batch-vs-single")
def test_ecef2aer_loop(benchmark):
    benchmark(loop_rows, rustmap3d.ecef2aer, ECEF_BATCH, *LLA_REF)


@pytest.mark.benchmark(group="rustmap3d-ecef2aer-batch-vs-single")
def test_ecef2aer_batch(benchmark):
    benchmark(rustmap3d.ecef2aer_batch, ECEF_BATCH, *LLA_REF)


"""Benchmark batched operations vs vectorized pymap3d equivalents"""


@pytest.mark.benchmark(group="ecef2lla-batch-vs-pymap3d")
def test_ecef2lla_batch_rustmap3d(benchmark):
    benchmark(rustmap3d.ecef2lla_batch, ECEF_BATCH)


@pytest.mark.benchmark(group="ecef2lla-batch-vs-pymap3d")
def test_ecef2geodetic_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.ecef2geodetic, *ECEF_BATCH.T)


@pytest.mark.benchmark(group="lla2ecef-batch-vs-pymap3d")
def test_lla2ecef_batch_rustmap3d(benchmark):
    benchmark(rustmap3d.lla2ecef_batch, LLA_BATCH)


@pytest.mark.benchmark(group="lla2ecef-batch-vs-pymap3d")
def test_geodetic2ecef_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.geodetic2ecef, *LLA_BATCH.T)


@pytest.mark.benchmark(group="ecef2aer-batch-vs-pymap3d")
def test_ecef2aer_batch_rustmap3d(benchmark):
    benchmark(
        lambda b, lat, lon, alt: rustmap3d.ecef2aer_batch(
            b, lat, lon, alt, parallel=True
        ),
        ECEF_BATCH,
        *LLA_REF,
    )


@pytest.mark.benchmark(group="ecef2aer-batch-vs-pymap3d")
def test_ecef2aer_vectorized_pymap3d(benchmark):
    benchmark(pymap3d.ecef2aer, *ECEF_BATCH.T, *LLA_REF)


"""Benchmark batch functions using parallel threads vs serial execution"""

AER_BATCH = numpy.array(
    [
        rustmap3d.ecef2aer(row[0], row[1], row[2], LLA_REF[0], LLA_REF[1], LLA_REF[2])
        for row in ECEF_BATCH
    ]
)
NED_BATCH = numpy.array(
    [
        rustmap3d.ecef2ned(row[0], row[1], row[2], LLA_REF[0], LLA_REF[1], LLA_REF[2])
        for row in ECEF_BATCH
    ]
)
ENU_BATCH = numpy.array(
    [
        rustmap3d.ecef2enu(row[0], row[1], row[2], LLA_REF[0], LLA_REF[1], LLA_REF[2])
        for row in ECEF_BATCH
    ]
)
INPUTS = {
    "ecef": ECEF_BATCH,
    "lla": LLA_BATCH,
    "enu": ENU_BATCH,
    "ned": NED_BATCH,
    "aer": AER_BATCH,
    "ecef_uvw": ECEF_BATCH,
}
REF2 = LLA_REF[:2]  # (lat, lon) only, for the uvw transforms
# transform name -> reference args passed after the batch
PAR_VS_SERIAL = {
    "ecef2lla": (),
    "lla2ecef": (),
    "ecef2aer": LLA_REF,
    "ecef2enu": LLA_REF,
    "ecef2ned": LLA_REF,
    "enu2ecef": LLA_REF,
    "enu2lla": LLA_REF,
    "enu2aer": (),
    "aer2ecef": LLA_REF,
    "aer2enu": (),
    "aer2ned": (),
    "lla2enu": LLA_REF,
    "lla2ned": LLA_REF,
    "ned2ecef": LLA_REF,
    "ned2lla": LLA_REF,
    "ned2aer": (),
    "aer2ecef_uvw": REF2,
    "enu2ecef_uvw": REF2,
    "ned2ecef_uvw": REF2,
    "ecef_uvw2aer": REF2,
    "ecef_uvw2enu": REF2,
    "ecef_uvw2ned": REF2,
}


@pytest.mark.parametrize(
    "mode",
    ["serial", "parallel"],
)
@pytest.mark.parametrize(
    "name",
    [
        pytest.param(
            name,
            marks=pytest.mark.benchmark(
                group=f"rustmap3d-{name}-batch-par-vs-serial",
            ),
        )
        for name in PAR_VS_SERIAL
    ],
)
def test_batch_par_vs_serial(benchmark, name, mode):
    fn = getattr(rustmap3d, f"{name}_batch")
    batch = INPUTS[name.split("2")[0]]
    args = PAR_VS_SERIAL[name]
    parallel = mode == "parallel"
    benchmark(fn, batch, *args, parallel=parallel)
