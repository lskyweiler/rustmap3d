//! Validates the transforms against the C kinematics-library: its published golden vectors, and the
//! line-for-line port in `reference.rs` swept over a lat/lon/alt grid with edge cases.
use super::reference::{self as kl, lla_deg2rad, lla_rad2deg};
use crate::{transforms::*, utils};
use rstest::*;

/// Angular agreement required with the reference, matching the C library's own tests.
const LATLON_TOL_RAD: f64 = 1e-12;
/// Positional agreement required with the reference. Bounded by this crate's rounded semi-minor
/// axis (6356752.314245 vs 6356752.3142451793), which moves positions by up to ~2e-7 m.
const POS_TOL_M: f64 = 1e-6;

const LATS_DEG: [f64; 7] = [-90.0, -89.9999, -45.0, 0.0, 45.0, 89.9999, 90.0];
const LONS_DEG: [f64; 7] = [-180.0, -179.9999, -90.0, 0.0, 90.0, 179.9999, 180.0];
const ALTS_M: [f64; 5] = [-10_000.0, 0.0, 10_000.0, 400_000.0, 36_000_000.0];

/// Every combination of `LATS_DEG`, `LONS_DEG` and `ALTS_M` [[degrees-degrees-meters]].
fn lla_grid() -> Vec<glam::DVec3> {
    let mut grid = Vec::new();
    for lat in LATS_DEG {
        for lon in LONS_DEG {
            for alt in ALTS_M {
                grid.push(glam::DVec3::new(lat, lon, alt));
            }
        }
    }
    return grid;
}

/// Difference between two angles in degrees, wrapped to [-180, 180).
fn ang_diff_deg(a: f64, b: f64) -> f64 {
    return (a - b + 180.0).rem_euclid(360.0) - 180.0;
}

fn assert_vec_near(actual: glam::DVec3, expected: glam::DVec3, tol: f64, ctx: &str) {
    let diff = (actual - expected).abs();
    assert!(
        diff.max_element() <= tol,
        "{ctx}: actual {actual:?} expected {expected:?} |diff| {diff:?} > tol {tol:e}"
    );
}

/// Compares LLAs in [[degrees-degrees-meters]]. Longitude is compared modulo 360 and skipped when
/// `check_lon` is false (it is undefined at the poles).
fn assert_lla_near(
    actual: glam::DVec3,
    expected: glam::DVec3,
    latlon_tol_rad: f64,
    alt_tol_m: f64,
    check_lon: bool,
    ctx: &str,
) {
    let dlat = f64::abs(actual.x - expected.x).to_radians();
    let dlon = if check_lon {
        f64::abs(ang_diff_deg(actual.y, expected.y)).to_radians()
    } else {
        0.0
    };
    let dalt = f64::abs(actual.z - expected.z);
    assert!(
        dlat <= latlon_tol_rad && dlon <= latlon_tol_rad && dalt <= alt_tol_m,
        "{ctx}: actual {actual:?} expected {expected:?} |dlat| {dlat:e} rad |dlon| {dlon:e} rad \
         |dalt| {dalt:e} m (tol {latlon_tol_rad:e} rad, {alt_tol_m:e} m)"
    );
}

/// Compares AERs in [[degrees-degrees-meters]], azimuth modulo 360.
///
/// `pos_noise_m` is how far apart the two underlying positions may legitimately be. It loosens
/// the angular tolerance by the angle that offset subtends: over the range for elevation, and over
/// the horizontal distance for azimuth, which becomes undefined looking straight up or down.
fn assert_aer_near(
    actual: glam::DVec3,
    expected: glam::DVec3,
    ang_tol_deg: f64,
    range_tol_m: f64,
    pos_noise_m: f64,
    ctx: &str,
) {
    let subtended_deg = |dist: f64| {
        if pos_noise_m == 0.0 {
            0.0
        } else {
            f64::to_degrees(pos_noise_m / dist)
        }
    };
    let horizontal = expected.z * f64::cos(expected.y.to_radians());
    let az_tol = ang_tol_deg + subtended_deg(f64::abs(horizontal));
    let el_tol = ang_tol_deg + subtended_deg(f64::abs(expected.z));

    let daz = f64::abs(ang_diff_deg(actual.x, expected.x));
    let del = f64::abs(actual.y - expected.y);
    let dr = f64::abs(actual.z - expected.z);
    assert!(
        daz <= az_tol && del <= el_tol && dr <= range_tol_m,
        "{ctx}: actual {actual:?} expected {expected:?} |daz| {daz:e} |del| {del:e} deg |dr| {dr:e} m \
         (tol az {az_tol:e} el {el_tol:e} deg, range {range_tol_m:e} m)"
    );
}

/// Oracle AER [[radians-radians-meters]], az in [-pi, pi] to this crate's convention
/// [[degrees-degrees-meters]], az in [0, 360].
fn aer_rad2deg(aer: glam::DVec3) -> glam::DVec3 {
    return glam::DVec3::new(
        utils::wrap_to_0_360(aer.x.to_degrees()),
        aer.y.to_degrees(),
        aer.z,
    );
}

fn enu2ned_axes(enu: glam::DVec3) -> glam::DVec3 {
    return glam::DVec3::new(enu.y, enu.x, -enu.z);
}

/// ECEF points that exercise `ecef2lla`: the grid, exact polar-axis and equatorial points, and the
/// C library's golden input.
fn ecef2lla_inputs() -> Vec<glam::DVec3> {
    let mut ecefs: Vec<glam::DVec3> = lla_grid()
        .into_iter()
        .map(|lla| kl::lla_to_ecef(lla_deg2rad(lla)))
        .collect();
    for z in [
        6356752.314245179,
        -6356752.314245179,
        7_000_000.0,
        -7_000_000.0,
        1.0,
        -1.0,
    ] {
        ecefs.push(glam::DVec3::new(0.0, 0.0, z));
    }
    for (x, y) in [
        (6378137.0, 0.0),
        (0.0, 6378137.0),
        (-7_000_000.0, 0.0),
        (0.0, -7_000_000.0),
    ] {
        ecefs.push(glam::DVec3::new(x, y, 0.0));
    }
    ecefs.push(glam::DVec3::new(4040000.0, 4813000.0, 1100000.0));
    return ecefs;
}

/// A target and reference for the local-frame conversions.
struct TargetRef {
    ecef: glam::DVec3,
    /// The target's true LLA [[degrees-degrees-meters]].
    lla: glam::DVec3,
    /// Reference LLA [[degrees-degrees-meters]].
    lla_ref: glam::DVec3,
}

/// Every grid point as a reference, each with a nearby target and the C library's ECEF golden
/// input as a far target.
fn target_refs() -> Vec<TargetRef> {
    let far_ecef = glam::DVec3::new(-1465000.0, 2880000.0, 5483500.0);
    // mid-latitude, where the reference ecef_to_lla is well conditioned
    let far_lla = lla_rad2deg(kl::ecef_to_lla(far_ecef));

    let mut pairs = Vec::new();
    for lla_ref in lla_grid() {
        let near = glam::DVec3::new(
            (lla_ref.x + 0.5).clamp(-90.0, 90.0),
            lla_ref.y - 0.7,
            lla_ref.z + 1000.0,
        );
        let near_ecef = kl::lla_to_ecef(lla_deg2rad(near));
        pairs.push(TargetRef {
            ecef: near_ecef,
            lla: near,
            lla_ref,
        });
        pairs.push(TargetRef {
            ecef: far_ecef,
            lla: far_lla,
            lla_ref,
        });
    }
    return pairs;
}

/// Local-frame vectors [[meters]] covering each axis, mixed directions, and large magnitudes.
fn local_vectors() -> Vec<glam::DVec3> {
    return vec![
        glam::DVec3::X * 1000.0,
        glam::DVec3::Y * 1000.0,
        glam::DVec3::Z * 1000.0,
        -glam::DVec3::X * 1000.0,
        -glam::DVec3::Y * 1000.0,
        -glam::DVec3::Z * 1000.0,
        glam::DVec3::new(1234.5, -6789.0, 4321.0),
        glam::DVec3::new(-46.43897628, -26.81155573, 44.99513264),
        glam::DVec3::new(-6500.0, -4800.0, 2400.0),
        glam::DVec3::new(3.0e7, -2.0e7, 1.0e7),
    ];
}

// --- Golden vectors from kinematics-library tests/cpp/transforms.test.cpp and tests/c/ ---

#[test]
fn golden_aer2ned() {
    // C input: (30 deg, 40 deg, -70) in radians-radians-meters
    let actual = aer2ned(glam::DVec3::new(30.0, 40.0, -70.0));
    let expected = glam::DVec3::new(-46.43897628, -26.81155573, 44.99513264);
    assert_vec_near(actual, expected, 1e-6, "aer_to_ned golden");
}

#[test]
fn golden_ecef2ned() {
    let lla_ref = lla_rad2deg(glam::DVec3::new(2.1, -1.1, 1250.0));
    let actual = ecef2ned(glam::DVec3::new(-1465000.0, 2880000.0, 5483500.0), lla_ref);
    let expected = glam::DVec3::new(2220.90854391709, 738.047215659648, -1205.58533167388);
    // the C test asks for 1e-8, below what the rounded semi-minor axis allows; see POS_TOL_M
    assert_vec_near(actual, expected, POS_TOL_M, "ecef_to_ned golden");
}

#[test]
fn golden_lla2ecef() {
    let actual = lla2ecef(lla_rad2deg(glam::DVec3::new(-0.8, 1.7, 4750.0)));
    let expected = glam::DVec3::new(-573960.23482409, 4417543.57131158, -4556021.90733023);
    assert_vec_near(actual, expected, POS_TOL_M, "lla_to_ecef golden");
}

#[test]
fn golden_lla2ned() {
    let lla = lla_rad2deg(glam::DVec3::new(-0.707, 0.496, 2400.0));
    let lla_ref = lla_rad2deg(glam::DVec3::new(-0.7, 0.5, 6000.0));
    let actual = ecef2ned(lla2ecef(lla), lla_ref);
    let expected = glam::DVec3::new(-44576.5574572138, -19432.24609638221, 3785.6580070712043);
    // the C test asks for 1e-14, finer than f64 resolves the ~6e6 m intermediate ECEF values
    assert_vec_near(actual, expected, POS_TOL_M, "lla_to_ned golden");
}

#[test]
fn golden_ned2aer() {
    let actual = ned2aer(glam::DVec3::new(-46.43897628, -26.81155573, 44.99513264));
    let expected = aer_rad2deg(glam::DVec3::new(-2.61799388, -0.6981317, 70.0));
    assert_aer_near(
        actual,
        expected,
        f64::to_degrees(1e-6),
        1e-6,
        0.0,
        "ned_to_aer golden",
    );
}

#[test]
fn golden_ned2ecef() {
    let lla_ref = lla_rad2deg(glam::DVec3::new(-2.9, -0.5, 6000.0));
    let actual = ned2ecef(glam::DVec3::new(-6500.0, -4800.0, 2400.0), lla_ref);
    let expected = glam::DVec3::new(-5442559.838214509, 2967814.4198892456, -1510590.1708055406);
    // the C test asks for 1e-14, finer than f64 resolves values of this magnitude
    assert_vec_near(actual, expected, POS_TOL_M, "ned_to_ecef golden");
}

#[test]
fn reference_port_reproduces_c_goldens() {
    // Checks the transcription in reference.rs against the C library's published results, at the
    // C tests' own tolerances except where noted.
    let v = glam::DVec3::new;
    let near = |a: glam::DVec3, e: glam::DVec3, tol: glam::DVec3, ctx: &str| {
        let diff = (a - e).abs();
        assert!(
            diff.cmple(tol).all(),
            "{ctx}: actual {a:?} expected {e:?} |diff| {diff:?}"
        );
    };
    let deg30_40 = v(30.0_f64.to_radians(), 40.0_f64.to_radians(), -70.0);
    near(
        kl::aer_to_ned(deg30_40),
        v(-46.43897628, -26.81155573, 44.99513264),
        v(1e-6, 1e-6, 1e-6),
        "aer_to_ned",
    );
    near(
        kl::ecef_to_lla(v(4040000.0, 4813000.0, 1100000.0)),
        v(0.174440593201424, 0.872492313234789, 1888.1851099917756),
        v(1e-12, 1e-12, 1e-4),
        "ecef_to_lla",
    );
    near(
        kl::ecef_to_ned(v(-1465000.0, 2880000.0, 5483500.0), v(2.1, -1.1, 1250.0)),
        v(2220.90854391709, 738.047215659648, -1205.58533167388),
        v(1e-8, 1e-8, 1e-8),
        "ecef_to_ned",
    );
    near(
        kl::lla_to_ecef(v(-0.8, 1.7, 4750.0)),
        v(-573960.23482409, 4417543.57131158, -4556021.90733023),
        v(1e-8, 1e-8, 1e-8),
        "lla_to_ecef",
    );
    near(
        kl::lla_to_ned(v(-0.707, 0.496, 2400.0), v(-0.7, 0.5, 6000.0)),
        v(-44576.5574572138, -19432.24609638221, 3785.6580070712043),
        // the C test's 1e-14 holds only with the exact constants in universal-constants; with
        // constants derived from a and 1/f the port lands 1.2e-9 m away
        v(1e-8, 1e-8, 1e-8),
        "lla_to_ned",
    );
    near(
        kl::ned_to_aer(v(-46.43897628, -26.81155573, 44.99513264)),
        v(-2.61799388, -0.6981317, 70.0),
        v(1e-6, 1e-6, 1e-6),
        "ned_to_aer",
    );
    near(
        kl::ned_to_ecef(v(-6500.0, -4800.0, 2400.0), v(-2.9, -0.5, 6000.0)),
        v(-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
        v(1e-8, 1e-8, 1e-8),
        "ned_to_ecef",
    );
    near(
        kl::ned_to_lla(v(12300.0, -8000.0, 6500.0), v(0.1, 2.5, 9200.0)),
        v(0.10194035135339555, 2.49873974993884, 2716.948501356757),
        v(1e-14, 1e-14, 1e-4),
        "ned_to_lla",
    );
}

// --- Sweeps against the reference port ---

#[test]
fn lla2ecef_matches_reference() {
    for lla in lla_grid() {
        let actual = lla2ecef(lla);
        let expected = kl::lla_to_ecef(lla_deg2rad(lla));
        assert_vec_near(actual, expected, POS_TOL_M, &format!("lla2ecef({lla:?})"));
    }
}

#[test]
fn ecef2lla_map3d_accuracy_by_altitude() {
    // A single Bowring-style correction step: matches the reference near the surface, but its
    // latitude error grows with altitude (measured: 3.5e-11 rad at 400 km, 1.5e-6 rad, ~60 m, at GEO).
    for lla in lla_grid() {
        let actual = ecef2lla_map3d(kl::lla_to_ecef(lla_deg2rad(lla)));
        let lat_tol_rad = match lla.z {
            alt if f64::abs(alt) <= 10_000.0 => LATLON_TOL_RAD,
            alt if alt <= 400_000.0 => 1e-10,
            _ => 2e-6,
        };
        let at_pole = f64::abs(lla.x) == 90.0;
        assert_lla_near(
            actual,
            lla,
            lat_tol_rad,
            1e-5,
            !at_pole,
            &format!("ecef2lla_map3d round trip {lla:?}"),
        );
    }
}

#[test]
fn ecef2ned_matches_reference() {
    for t in target_refs() {
        let actual = ecef2ned(t.ecef, t.lla_ref);
        let expected = kl::ecef_to_ned(t.ecef, lla_deg2rad(t.lla_ref));
        assert_vec_near(
            actual,
            expected,
            POS_TOL_M,
            &format!("ecef2ned({:?}, {:?})", t.ecef, t.lla_ref),
        );
    }
}

#[test]
fn ned2ecef_matches_reference() {
    for lla_ref in lla_grid() {
        for ned in local_vectors() {
            let actual = ned2ecef(ned, lla_ref);
            let expected = kl::ned_to_ecef(ned, lla_deg2rad(lla_ref));
            assert_vec_near(
                actual,
                expected,
                POS_TOL_M,
                &format!("ned2ecef({ned:?}, {lla_ref:?})"),
            );
        }
    }
}

#[test]
fn ned_uvw_matches_reference() {
    for lla_ref in lla_grid() {
        for v in local_vectors() {
            let ctx = format!("{v:?} at {lla_ref:?}");
            let expected_ned = kl::ecef_to_ned_local(v, lla_deg2rad(lla_ref));
            assert_vec_near(
                ecef_uvw2ned(v, lla_ref),
                expected_ned,
                POS_TOL_M,
                &format!("ecef_uvw2ned {ctx}"),
            );
            let expected_ecef = kl::ned_to_ecef_local(v, lla_deg2rad(lla_ref));
            assert_vec_near(
                ned2ecef_uvw(v, lla_ref),
                expected_ecef,
                POS_TOL_M,
                &format!("ned2ecef_uvw {ctx}"),
            );
        }
    }
}

#[test]
fn enu_matches_reference() {
    // the reference has no ENU; ENU is NED with north/east swapped and down negated
    for t in target_refs() {
        let actual = enu2ned_axes(ecef2enu(t.ecef, t.lla_ref));
        let expected = kl::ecef_to_ned(t.ecef, lla_deg2rad(t.lla_ref));
        assert_vec_near(
            actual,
            expected,
            POS_TOL_M,
            &format!("ecef2enu({:?}, {:?})", t.ecef, t.lla_ref),
        );
    }
    for lla_ref in lla_grid() {
        for v in local_vectors() {
            let ctx = format!("{v:?} at {lla_ref:?}");
            let ned = enu2ned_axes(v);
            let expected = kl::ned_to_ecef(ned, lla_deg2rad(lla_ref));
            assert_vec_near(
                enu2ecef(v, lla_ref),
                expected,
                POS_TOL_M,
                &format!("enu2ecef {ctx}"),
            );
            let expected = kl::ned_to_ecef_local(ned, lla_deg2rad(lla_ref));
            assert_vec_near(
                enu2ecef_uvw(v, lla_ref),
                expected,
                POS_TOL_M,
                &format!("enu2ecef_uvw {ctx}"),
            );
            let expected = kl::ecef_to_ned_local(v, lla_deg2rad(lla_ref));
            assert_vec_near(
                enu2ned_axes(ecef_uvw2enu(v, lla_ref)),
                expected,
                POS_TOL_M,
                &format!("ecef_uvw2enu {ctx}"),
            );
        }
    }
}

#[test]
fn local_aer_matches_reference() {
    let ang_tol_deg = LATLON_TOL_RAD.to_degrees();
    let mut vectors = local_vectors();
    vectors.push(glam::DVec3::ZERO);
    for ned in vectors {
        let expected = aer_rad2deg(kl::ned_to_aer(ned));
        assert_aer_near(
            ned2aer(ned),
            expected,
            ang_tol_deg,
            POS_TOL_M,
            0.0,
            &format!("ned2aer({ned:?})"),
        );
        let enu = glam::DVec3::new(ned.y, ned.x, -ned.z);
        assert_aer_near(
            enu2aer(enu),
            expected,
            ang_tol_deg,
            POS_TOL_M,
            0.0,
            &format!("enu2aer({enu:?})"),
        );
    }
    for az in [-150.0, 0.0, 30.0, 90.0, 179.0, 270.0, 359.0] {
        for el in [-90.0, -40.0, 0.0, 40.0, 90.0] {
            for range in [-70.0, 0.0, 1.0, 1.0e7] {
                let aer = glam::DVec3::new(az, el, range);
                let expected = kl::aer_to_ned(glam::DVec3::new(
                    f64::to_radians(az),
                    f64::to_radians(el),
                    range,
                ));
                assert_vec_near(
                    aer2ned(aer),
                    expected,
                    POS_TOL_M,
                    &format!("aer2ned({aer:?})"),
                );
                assert_vec_near(
                    enu2ned_axes(aer2enu(aer)),
                    expected,
                    POS_TOL_M,
                    &format!("aer2enu({aer:?})"),
                );
            }
        }
    }
}

#[test]
fn ecef_aer_matches_reference() {
    let ang_tol_deg = LATLON_TOL_RAD.to_degrees();
    for t in target_refs() {
        let ctx = format!("{:?} at {:?}", t.ecef, t.lla_ref);
        let ref_rad = lla_deg2rad(t.lla_ref);
        let expected = aer_rad2deg(kl::ned_to_aer(kl::ecef_to_ned(t.ecef, ref_rad)));
        assert_aer_near(
            ecef2aer(t.ecef, t.lla_ref),
            expected,
            ang_tol_deg,
            POS_TOL_M,
            POS_TOL_M,
            &format!("ecef2aer {ctx}"),
        );

        let expected = aer_rad2deg(kl::ned_to_aer(kl::ecef_to_ned_local(t.ecef, ref_rad)));
        // rotating the vector rounds each component by about 1e-15 of its magnitude
        let rotation_noise_m = 1e-15 * t.ecef.length();
        assert_aer_near(
            ecef_uvw2aer(t.ecef, t.lla_ref),
            expected,
            ang_tol_deg,
            POS_TOL_M,
            rotation_noise_m,
            &format!("ecef_uvw2aer {ctx}"),
        );
    }
    for lla_ref in lla_grid() {
        let ref_rad = lla_deg2rad(lla_ref);
        for aer in [
            glam::DVec3::new(30.0, 40.0, 70.0),
            glam::DVec3::new(210.0, -10.0, 5.0e5),
        ] {
            let ctx = format!("{aer:?} at {lla_ref:?}");
            let ned = kl::aer_to_ned(glam::DVec3::new(
                aer.x.to_radians(),
                aer.y.to_radians(),
                aer.z,
            ));
            assert_vec_near(
                aer2ecef(aer, lla_ref),
                kl::ned_to_ecef(ned, ref_rad),
                POS_TOL_M,
                &format!("aer2ecef {ctx}"),
            );
            assert_vec_near(
                aer2ecef_uvw(aer, lla_ref),
                kl::ned_to_ecef_local(ned, ref_rad),
                POS_TOL_M,
                &format!("aer2ecef_uvw {ctx}"),
            );
        }
    }
}

#[test]
fn dcms_match_reference() {
    for lla_ref in lla_grid() {
        let ref_rad = lla_deg2rad(lla_ref);
        for v in [glam::DVec3::X, glam::DVec3::Y, glam::DVec3::Z] {
            let ctx = format!("{v:?} at {lla_ref:?}");
            let ned = kl::ecef_to_ned_local(v, ref_rad);
            assert_vec_near(
                ecef2ned_dcm(lla_ref) * v,
                ned,
                1e-15,
                &format!("ecef2ned_dcm {ctx}"),
            );
            assert_vec_near(
                ecef2enu_dcm(lla_ref) * v,
                glam::DVec3::new(ned.y, ned.x, -ned.z),
                1e-15,
                &format!("ecef2enu_dcm {ctx}"),
            );
            let ecef = kl::ned_to_ecef_local(v, ref_rad);
            assert_vec_near(
                ned2ecef_dcm(lla_ref) * v,
                ecef,
                1e-15,
                &format!("ned2ecef_dcm {ctx}"),
            );
            let ecef = kl::ned_to_ecef_local(enu2ned_axes(v), ref_rad);
            assert_vec_near(
                enu2ecef_dcm(lla_ref) * v,
                ecef,
                1e-15,
                &format!("enu2ecef_dcm {ctx}"),
            );
        }
    }
}

// --- Self-consistency (no reference needed) ---

#[test]
fn local_frames_round_trip() {
    for t in target_refs() {
        let (ecef, lla_ref) = (t.ecef, t.lla_ref);
        let ctx = format!("{ecef:?} at {lla_ref:?}");
        assert_vec_near(
            enu2ecef(ecef2enu(ecef, lla_ref), lla_ref),
            ecef,
            POS_TOL_M,
            &format!("enu {ctx}"),
        );
        assert_vec_near(
            ned2ecef(ecef2ned(ecef, lla_ref), lla_ref),
            ecef,
            POS_TOL_M,
            &format!("ned {ctx}"),
        );
        assert_vec_near(
            aer2ecef(ecef2aer(ecef, lla_ref), lla_ref),
            ecef,
            POS_TOL_M,
            &format!("aer {ctx}"),
        );
        assert_vec_near(
            enu2ecef_uvw(ecef_uvw2enu(ecef, lla_ref), lla_ref),
            ecef,
            POS_TOL_M,
            &format!("enu uvw {ctx}"),
        );
        assert_vec_near(
            ned2ecef_uvw(ecef_uvw2ned(ecef, lla_ref), lla_ref),
            ecef,
            POS_TOL_M,
            &format!("ned uvw {ctx}"),
        );
        assert_vec_near(
            aer2ecef_uvw(ecef_uvw2aer(ecef, lla_ref), lla_ref),
            ecef,
            POS_TOL_M,
            &format!("aer uvw {ctx}"),
        );
    }
}

#[test]
fn local_frames_agree_with_each_other() {
    for t in target_refs() {
        let ctx = format!("{:?} at {:?}", t.ecef, t.lla_ref);
        let ned = ecef2ned(t.ecef, t.lla_ref);
        assert_vec_near(
            enu2ned_axes(ecef2enu(t.ecef, t.lla_ref)),
            ned,
            POS_TOL_M,
            &format!("enu vs ned {ctx}"),
        );
        assert_aer_near(
            ecef2aer(t.ecef, t.lla_ref),
            ned2aer(ned),
            1e-9,
            POS_TOL_M,
            0.0,
            &format!("aer vs ned {ctx}"),
        );
    }
}

#[test]
fn dcms_are_proper_rotations() {
    for lla_ref in lla_grid() {
        for (name, dcm) in [
            ("ecef2enu", ecef2enu_dcm(lla_ref)),
            ("enu2ecef", enu2ecef_dcm(lla_ref)),
            ("ecef2ned", ecef2ned_dcm(lla_ref)),
            ("ned2ecef", ned2ecef_dcm(lla_ref)),
        ] {
            let ctx = format!("{name}_dcm({lla_ref:?})");
            assert!(
                (dcm * dcm.transpose()).abs_diff_eq(glam::DMat3::IDENTITY, 1e-14),
                "{ctx} not orthonormal: {dcm:?}"
            );
            assert!(
                f64::abs(dcm.determinant() - 1.0) <= 1e-14,
                "{ctx} determinant != 1: {dcm:?}"
            );
        }
        let ctx = format!("at {lla_ref:?}");
        assert!(
            glam::DMat3::from_quat(ecef2enu_quat(lla_ref))
                .abs_diff_eq(ecef2enu_dcm(lla_ref), 1e-15),
            "enu quat vs dcm {ctx}"
        );
        assert!(
            glam::DMat3::from_quat(ecef2ned_quat(lla_ref))
                .abs_diff_eq(ecef2ned_dcm(lla_ref), 1e-15),
            "ned quat vs dcm {ctx}"
        );
    }
}
