"""The NumPy batch functions must match the single-point functions exactly, row by row."""

import numpy as np
import pytest

import rustmap3d

RNG = np.random.default_rng(0)
N = 500


def _with_edges(random_rows, edges):
    return np.vstack([random_rows, np.asarray(edges, dtype=np.float64)])


LLA = _with_edges(
    np.column_stack(
        [
            RNG.uniform(-90, 90, N),
            RNG.uniform(-180, 180, N),
            RNG.uniform(-10_000, 40_000_000, N),
        ]
    ),
    [
        (90.0, 0.0, 0.0),
        (-90.0, 180.0, 10_000.0),
        (0.0, -180.0, 0.0),
        (89.9999, 179.9999, 36_000_000.0),
        (np.nan, np.nan, np.nan),
    ],
)
ECEF = _with_edges(
    np.array([rustmap3d.lla2ecef(*row) for row in LLA[:N]]),
    [
        (0.0, 0.0, 6356752.314245179),
        (0.0, 0.0, -6356752.314245179),
        (0.0, 0.0, 0.0),
        (np.nan, np.nan, np.nan),
    ],
)
VECTORS = _with_edges(
    RNG.uniform(-1, 1, (N, 3)) * 10 ** RNG.uniform(-3, 7.5, (N, 1)),
    [
        (1000.0, 0.0, 0.0),
        (0.0, 0.0, -1000.0),
        (0.0, 0.0, 0.0),
        (np.nan, np.nan, np.nan),
    ],
)
AER = _with_edges(
    np.column_stack(
        [
            RNG.uniform(-360, 720, N),
            RNG.uniform(-90, 90, N),
            RNG.uniform(0, 4e7, N),
        ]
    ),
    [(0.0, 90.0, 1000.0), (360.0, 0.0, 1.0), (30.0, 40.0, -70.0), (0.0, 0.0, 0.0)],
)

REFS = [
    (0.0, 0.0, 0.0),
    (38.9, -77.0, 120.0),
    (90.0, 0.0, 0.0),
    (60.0, 179.9999, 36_000_000.0),
]

# (function name, inputs, reference arguments: None, "ll" for lat/lon, "lla" for lat/lon/alt)
CASES = [
    ("ecef2lla", ECEF, None),
    ("lla2ecef", LLA, None),
    ("enu2aer", VECTORS, None),
    ("aer2enu", AER, None),
    ("ned2aer", VECTORS, None),
    ("aer2ned", AER, None),
    ("ecef_uvw2enu", VECTORS, "ll"),
    ("enu2ecef_uvw", VECTORS, "ll"),
    ("ecef_uvw2ned", VECTORS, "ll"),
    ("ned2ecef_uvw", VECTORS, "ll"),
    ("ecef_uvw2aer", VECTORS, "ll"),
    ("aer2ecef_uvw", AER, "ll"),
    ("ecef2enu", ECEF, "lla"),
    ("lla2enu", LLA, "lla"),
    ("enu2ecef", VECTORS, "lla"),
    ("enu2lla", VECTORS, "lla"),
    ("ecef2ned", ECEF, "lla"),
    ("lla2ned", LLA, "lla"),
    ("ned2ecef", VECTORS, "lla"),
    ("ned2lla", VECTORS, "lla"),
    ("ecef2aer", ECEF, "lla"),
    ("aer2ecef", AER, "lla"),
]


def _ref_args(kind, ref):
    if kind is None:
        return (), {}
    if kind == "ll":
        return ref[:2], {}
    return ref[:2], {"alt_ref_m": ref[2]}


def _cases_with_refs():
    for name, inputs, kind in CASES:
        refs = [None] if kind is None else REFS
        for ref in refs:
            yield pytest.param(name, inputs, kind, ref, id=f"{name}-{ref}")


@pytest.mark.parametrize("name,inputs,kind,ref", list(_cases_with_refs()))
def test_batch_matches_single(name, inputs, kind, ref):
    args, kwargs = _ref_args(kind, ref)
    single = getattr(rustmap3d, name)
    batch = getattr(rustmap3d, f"{name}_batch")

    expected = np.array([single(*row, *args, **kwargs) for row in inputs])
    actual = batch(inputs, *args, **kwargs)

    assert actual.shape == inputs.shape
    assert actual.dtype == np.float64
    # exact, NaN-aware; also distinguishes 0.0 from -0.0
    np.testing.assert_array_equal(actual.view(np.uint64), expected.view(np.uint64))


@pytest.mark.parametrize(
    "name,inputs,kind", [c[:3] for c in CASES], ids=[c[0] for c in CASES]
)
def test_batch_empty_and_single_row(name, inputs, kind):
    args, kwargs = _ref_args(kind, REFS[1])
    batch = getattr(rustmap3d, f"{name}_batch")

    empty = batch(np.empty((0, 3)), *args, **kwargs)
    assert empty.shape == (0, 3)

    one = batch(inputs[:1], *args, **kwargs)
    expected = getattr(rustmap3d, name)(*inputs[0], *args, **kwargs)
    np.testing.assert_array_equal(one, [expected])


def test_batch_alt_ref_defaults_to_zero():
    with_default = rustmap3d.ecef2ned_batch(ECEF, 38.9, -77.0)
    explicit = rustmap3d.ecef2ned_batch(ECEF, 38.9, -77.0, alt_ref_m=0.0)
    np.testing.assert_array_equal(with_default, explicit)


def test_batch_accepts_non_contiguous_input():
    wide = np.zeros((len(ECEF), 6))
    wide[:, ::2] = ECEF
    strided = wide[:, ::2]
    assert not strided.flags["C_CONTIGUOUS"]
    np.testing.assert_array_equal(
        rustmap3d.ecef2lla_batch(strided), rustmap3d.ecef2lla_batch(ECEF)
    )


def test_batch_does_not_modify_input():
    before = ECEF.copy()
    rustmap3d.ecef2lla_batch(ECEF)
    np.testing.assert_array_equal(ECEF, before)


@pytest.mark.parametrize("shape", [(5,), (5, 2), (5, 4), (2, 5, 3), ()])
def test_batch_rejects_wrong_shape(shape):
    with pytest.raises(ValueError, match=r"must have shape \(N, 3\)"):
        rustmap3d.ecef2lla_batch(np.zeros(shape))


@pytest.mark.parametrize("dtype", [np.float32, np.int64, np.int32])
def test_batch_converts_other_dtypes_to_float64(dtype):
    converted = ECEF[:N].astype(dtype)
    np.testing.assert_array_equal(
        rustmap3d.ecef2lla_batch(converted),
        rustmap3d.ecef2lla_batch(converted.astype(np.float64)),
    )


def test_batch_accepts_nested_lists():
    rows = ECEF[:10].tolist()
    np.testing.assert_array_equal(
        rustmap3d.ecef2lla_batch(rows), rustmap3d.ecef2lla_batch(ECEF[:10])
    )


def test_batch_rejects_non_numeric():
    with pytest.raises((ValueError, TypeError)):
        rustmap3d.ecef2lla_batch([["a", "b", "c"]])


def test_thread_scope():
    with rustmap3d.threadpool_limits(1):
        rustmap3d.ecef2lla_batch(ECEF)


def test_global_thread():
    rustmap3d.set_global_thread_limits(1)


def test_global_thread_raises():
    with pytest.raises(OverflowError):
        rustmap3d.set_global_thread_limits(-1)

    with pytest.raises(RuntimeError):
        rustmap3d.set_global_thread_limits(100000)
