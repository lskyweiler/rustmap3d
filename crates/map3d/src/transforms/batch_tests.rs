//! The batched transforms must reproduce the single-point transforms bit for bit.
use crate::transforms::*;

/// Deterministic pseudo-random values in [0, 1) (SplitMix64), so failures reproduce exactly.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next_f64(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^= z >> 31;
        return (z >> 11) as f64 / (1u64 << 53) as f64;
    }

    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        return lo + (hi - lo) * self.next_f64();
    }
}

const N_RANDOM: usize = 1000;

/// LLAs [[degrees-degrees-meters]]: random points plus poles, equator, antimeridian, and extremes.
fn lla_inputs() -> Vec<glam::DVec3> {
    let mut rng = SplitMix64(1);
    let mut v: Vec<glam::DVec3> = (0..N_RANDOM)
        .map(|_| {
            glam::DVec3::new(
                rng.uniform(-90.0, 90.0),
                rng.uniform(-180.0, 180.0),
                rng.uniform(-10_000.0, 40_000_000.0),
            )
        })
        .collect();
    for lat in [-90.0, -89.9999, 0.0, 89.9999, 90.0] {
        for lon in [-180.0, 0.0, 179.9999, 180.0] {
            for alt in [-10_000.0, 0.0, 400_000.0, 36_000_000.0] {
                v.push(glam::DVec3::new(lat, lon, alt));
            }
        }
    }
    v.push(glam::DVec3::splat(f64::NAN));
    return v;
}

/// ECEF positions [[meters]]: the LLA inputs converted, plus the polar axis, origin, and NaN.
fn ecef_inputs() -> Vec<glam::DVec3> {
    let mut v: Vec<glam::DVec3> = lla_inputs().into_iter().map(|lla| lla2ecef(lla)).collect();
    for z in [6356752.314245179, -6356752.314245179, 1.0, -1.0] {
        v.push(glam::DVec3::new(0.0, 0.0, z));
    }
    v.push(glam::DVec3::ZERO);
    return v;
}

/// Local-frame or direction vectors [[meters]]: random, axis-aligned, zero, and NaN.
fn vector_inputs() -> Vec<glam::DVec3> {
    let mut rng = SplitMix64(2);
    let mut v: Vec<glam::DVec3> = (0..N_RANDOM)
        .map(|_| {
            let scale = 10f64.powf(rng.uniform(-3.0, 7.5));
            let dir = glam::DVec3::new(
                rng.uniform(-1.0, 1.0),
                rng.uniform(-1.0, 1.0),
                rng.uniform(-1.0, 1.0),
            );
            dir * scale
        })
        .collect();
    for axis in [glam::DVec3::X, glam::DVec3::Y, glam::DVec3::Z] {
        v.push(axis * 1000.0);
        v.push(-axis * 1000.0);
    }
    v.push(glam::DVec3::ZERO);
    v.push(glam::DVec3::splat(f64::NAN));
    return v;
}

/// Reference points [[degrees-degrees-meters]] shared by every point in a batch.
fn lla_refs() -> Vec<glam::DVec3> {
    return vec![
        glam::DVec3::new(0.0, 0.0, 0.0),
        glam::DVec3::new(38.9, -77.0, 120.0),
        glam::DVec3::new(-33.9, 151.2, -25.0),
        glam::DVec3::new(90.0, 0.0, 0.0),
        glam::DVec3::new(-90.0, 180.0, 10_000.0),
        glam::DVec3::new(60.0, 179.9999, 36_000_000.0),
    ];
}

/// Compares the bits of every component, so -0.0 vs 0.0 and differing NaN payloads both count.
fn assert_bits_eq(actual: &[glam::DVec3], expected: &[glam::DVec3], ctx: &str) {
    assert_eq!(actual.len(), expected.len(), "{ctx}: length");
    for (i, (a, e)) in actual.iter().zip(expected).enumerate() {
        let a_bits = a.to_array().map(f64::to_bits);
        let e_bits = e.to_array().map(f64::to_bits);
        assert_eq!(
            a_bits, e_bits,
            "{ctx}: index {i}: batch {a:?} != single {e:?}"
        );
    }
}

/// Checks a batched transform and its `_into` variant against the single-point transform on
/// `inputs`, on an empty batch, and on a one-element batch.
fn check_batch(
    ctx: &str,
    inputs: &[glam::DVec3],
    single: impl Fn(glam::DVec3) -> glam::DVec3,
    batch: impl Fn(&[glam::DVec3]) -> Vec<glam::DVec3>,
    batch_into: impl Fn(&[glam::DVec3], &mut [glam::DVec3]),
) {
    let expected: Vec<glam::DVec3> = inputs.iter().map(|&v| single(v)).collect();
    assert_bits_eq(&batch(inputs), &expected, &format!("{ctx} batch"));

    // pre-fill with a sentinel so an unwritten slot can't pass by accident
    let mut out = vec![glam::DVec3::splat(-12345.0); inputs.len()];
    batch_into(inputs, &mut out);
    assert_bits_eq(&out, &expected, &format!("{ctx} batch_into"));

    assert!(batch(&[]).is_empty(), "{ctx}: empty batch");
    batch_into(&[], &mut []);
    assert_bits_eq(
        &batch(&inputs[..1]),
        &expected[..1],
        &format!("{ctx} one element"),
    );
}

// --- No shared argument ---

#[test]
fn ecef2lla_batch_matches_single() {
    check_batch(
        "ecef2lla",
        &ecef_inputs(),
        |v| ecef2lla(v),
        ecef2lla_batch,
        ecef2lla_batch_into,
    );
}

#[test]
fn lla2ecef_batch_matches_single() {
    check_batch(
        "lla2ecef",
        &lla_inputs(),
        |v| lla2ecef(v),
        lla2ecef_batch,
        lla2ecef_batch_into,
    );
}

// --- Shared reference point ---

/// Runs `check_batch` once per shared reference in `lla_refs`.
macro_rules! check_with_ref {
    ($name:literal, $inputs:expr, $single:ident, $batch:ident, $batch_into:ident) => {
        let inputs = $inputs;
        for r in lla_refs() {
            check_batch(
                &format!("{} ref {r:?}", $name),
                &inputs,
                |v| $single(v, r),
                |b| $batch(b, r),
                |b, out| $batch_into(b, r, out),
            );
        }
    };
}

#[test]
fn ecef2enu_batch_matches_single() {
    check_with_ref!(
        "ecef2enu",
        ecef_inputs(),
        ecef2enu,
        ecef2enu_batch,
        ecef2enu_batch_into
    );
}

#[test]
fn enu2ecef_batch_matches_single() {
    check_with_ref!(
        "enu2ecef",
        vector_inputs(),
        enu2ecef,
        enu2ecef_batch,
        enu2ecef_batch_into
    );
}

#[test]
fn ecef_uvw2enu_batch_matches_single() {
    check_with_ref!(
        "ecef_uvw2enu",
        vector_inputs(),
        ecef_uvw2enu,
        ecef_uvw2enu_batch,
        ecef_uvw2enu_batch_into
    );
}

#[test]
fn enu2ecef_uvw_batch_matches_single() {
    check_with_ref!(
        "enu2ecef_uvw",
        vector_inputs(),
        enu2ecef_uvw,
        enu2ecef_uvw_batch,
        enu2ecef_uvw_batch_into
    );
}

#[test]
fn ecef2ned_batch_matches_single() {
    check_with_ref!(
        "ecef2ned",
        ecef_inputs(),
        ecef2ned,
        ecef2ned_batch,
        ecef2ned_batch_into
    );
}

#[test]
fn ned2ecef_batch_matches_single() {
    check_with_ref!(
        "ned2ecef",
        vector_inputs(),
        ned2ecef,
        ned2ecef_batch,
        ned2ecef_batch_into
    );
}

#[test]
fn ecef_uvw2ned_batch_matches_single() {
    check_with_ref!(
        "ecef_uvw2ned",
        vector_inputs(),
        ecef_uvw2ned,
        ecef_uvw2ned_batch,
        ecef_uvw2ned_batch_into
    );
}

#[test]
fn ned2ecef_uvw_batch_matches_single() {
    check_with_ref!(
        "ned2ecef_uvw",
        vector_inputs(),
        ned2ecef_uvw,
        ned2ecef_uvw_batch,
        ned2ecef_uvw_batch_into
    );
}

// --- Output length checks ---

#[test]
#[should_panic(expected = "length")]
fn batch_into_rejects_short_output() {
    let inputs = ecef_inputs();
    let mut out = vec![glam::DVec3::ZERO; inputs.len() - 1];
    ecef2lla_batch_into(&inputs, &mut out);
}

#[test]
#[should_panic(expected = "length")]
fn batch_into_rejects_long_output() {
    let mut out = vec![glam::DVec3::ZERO; lla_inputs().len() + 1];
    lla2ecef_batch_into(&lla_inputs(), &mut out);
}
