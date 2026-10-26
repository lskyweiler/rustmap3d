import random

import pymap3d.vincenty

import rustmap3d

ITERATIONS = 1000
ROUNDS = 10
N = 10

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


def iter_points(fn_to_test, points):
    for p in points:
        fn_to_test(*p)


def test_vincenty_direct_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.vincenty_direct, direct_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


def test_vreckon_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.vincenty.vreckon, direct_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


def test_vincenty_inverse_rustmap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(rustmap3d.vincenty_inverse, inverse_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )


def test_vdist_pymap3d(benchmark):
    benchmark.pedantic(
        iter_points,
        args=(pymap3d.vincenty.vdist, inverse_points),
        iterations=ITERATIONS,
        rounds=ROUNDS,
    )
