import pymap3d

import rustmap3d

ITERATIONS = 1000
ROUNDS = 10
N = 10
ecef_points = [rustmap3d.rand_ecef() for _ in range(N)]
lla_points = [rustmap3d.rand_lla() for _ in range(N)]


# we want to iterate over the exact same points each bench
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
