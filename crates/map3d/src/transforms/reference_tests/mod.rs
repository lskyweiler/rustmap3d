//! Validates the transforms against the C kinematics-library: its published golden vectors, and
//! recorded outputs of a reference implementation (`reference_vectors.rs`) over a lat/lon/alt grid
//! with edge cases.
#[rustfmt::skip]
mod reference_vectors;

use crate::{transforms::*, utils};
use reference_vectors::*;
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

/// AER [[radians-radians-meters]], az in [-pi, pi], as the golden vectors give it, to this crate's
/// convention [[degrees-degrees-meters]], az in [0, 360].
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

/// LLA [[radians-radians-meters]], as the golden vectors give it, to [[degrees-degrees-meters]].
fn lla_rad2deg(lla_rad: glam::DVec3) -> glam::DVec3 {
    return glam::DVec3::new(lla_rad.x.to_degrees(), lla_rad.y.to_degrees(), lla_rad.z);
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
fn golden_ecef2lla() {
    let actual = ecef2lla(glam::DVec3::new(4040000.0, 4813000.0, 1100000.0));
    // altitude from kinematics-library python/tests/ecef_to_lla_test.py, which carries more digits
    let expected = lla_rad2deg(glam::DVec3::new(
        0.174440593201424,
        0.872492313234789,
        1888.1851099917756,
    ));
    assert_lla_near(actual, expected, 1e-12, 1e-4, true, "ecef_to_lla golden");
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
fn golden_ned2lla() {
    let lla_ref = lla_rad2deg(glam::DVec3::new(0.1, 2.5, 9200.0));
    let actual = ecef2lla(ned2ecef(
        glam::DVec3::new(12300.0, -8000.0, 6500.0),
        lla_ref,
    ));
    let expected = lla_rad2deg(glam::DVec3::new(
        0.10194035135339555,
        2.49873974993884,
        2716.948501356757,
    ));
    assert_lla_near(actual, expected, 1e-14, 1e-4, true, "ned_to_lla golden");
}

// --- Sweeps against the recorded reference outputs ---

#[test]
fn lla2ecef_matches_reference() {
    for row in LLA2ECEF {
        let actual = lla2ecef(row.lla);
        assert_vec_near(
            actual,
            row.ecef,
            POS_TOL_M,
            &format!("lla2ecef({:?})", row.lla),
        );
    }
}

#[rstest]
#[case::ferrari(|v| ecef2lla_ferarri(v))]
#[case::default(|v| ecef2lla(v))]
fn ecef2lla_matches_reference(#[case] under_test: fn(glam::DVec3) -> glam::DVec3) {
    // ECEF2LLA leaves out the polar-axis points where the reference returned NaN; those are
    // covered against their true LLA by ecef2lla_near_polar_axis_round_trips
    for row in ECEF2LLA {
        let actual = under_test(row.ecef);
        assert_lla_near(
            actual,
            row.lla,
            LATLON_TOL_RAD,
            POS_TOL_M,
            true,
            &format!("ecef2lla({:?})", row.ecef),
        );
    }
}

#[test]
fn ecef2lla_map3d_accuracy_by_altitude() {
    // A single Bowring-style correction step: matches the reference near the surface, but its
    // latitude error grows with altitude (measured: 3.5e-11 rad at 400 km, 1.5e-6 rad, ~60 m, at GEO).
    for row in LLA2ECEF {
        let lla = row.lla;
        let actual = ecef2lla_map3d(row.ecef);
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
fn ecef2lla_inverts_lla2ecef() {
    for lla in lla_grid() {
        let actual = ecef2lla(lla2ecef(lla));
        let at_pole = f64::abs(lla.x) == 90.0;
        assert_lla_near(
            actual,
            lla,
            LATLON_TOL_RAD,
            POS_TOL_M,
            !at_pole,
            &format!("round trip {lla:?}"),
        );
    }
}

#[test]
fn ecef2lla_near_polar_axis_round_trips() {
    for row in NEAR_POLE {
        let actual = ecef2lla(row.ecef);
        let ctx = format!("ecef2lla({:?}) from {:?}", row.ecef, row.lla);
        assert!(actual.is_finite(), "{ctx}: not finite {actual:?}");
        // longitude is only well conditioned once the point is measurably off the axis
        let check_lon = row.offset_deg >= 1e-6;
        assert_lla_near(actual, row.lla, LATLON_TOL_RAD, POS_TOL_M, check_lon, &ctx);
    }
}

#[test]
fn ecef2lla_polar_axis_is_plus_minus_90() {
    let north = ecef2lla(glam::DVec3::new(0.0, 0.0, 6356752.314245179));
    let south = ecef2lla(glam::DVec3::new(0.0, 0.0, -6356752.314245179));
    assert_eq!(north.x, 90.0, "north pole latitude: {north:?}");
    assert_eq!(south.x, -90.0, "south pole latitude: {south:?}");
    assert!(
        f64::abs(north.z) <= POS_TOL_M,
        "north pole altitude: {north:?}"
    );
    assert!(
        f64::abs(south.z) <= POS_TOL_M,
        "south pole altitude: {south:?}"
    );
}

#[test]
fn ecef2ned_matches_reference() {
    for t in TARGET_REFS {
        let actual = ecef2ned(t.ecef, t.lla_ref);
        assert_vec_near(
            actual,
            t.ned,
            POS_TOL_M,
            &format!("ecef2ned({:?}, {:?})", t.ecef, t.lla_ref),
        );
    }
}

#[test]
fn ned2ecef_matches_reference() {
    for row in LOCAL_VECTORS {
        let actual = ned2ecef(row.local, row.lla_ref);
        assert_vec_near(
            actual,
            row.ned2ecef,
            POS_TOL_M,
            &format!("ned2ecef({:?}, {:?})", row.local, row.lla_ref),
        );
    }
}

#[test]
fn ned_uvw_matches_reference() {
    for row in LOCAL_VECTORS {
        let ctx = format!("{:?} at {:?}", row.local, row.lla_ref);
        assert_vec_near(
            ecef_uvw2ned(row.local, row.lla_ref),
            row.ecef_uvw2ned,
            POS_TOL_M,
            &format!("ecef_uvw2ned {ctx}"),
        );
        assert_vec_near(
            ned2ecef_uvw(row.local, row.lla_ref),
            row.ned2ecef_uvw,
            POS_TOL_M,
            &format!("ned2ecef_uvw {ctx}"),
        );
    }
}

#[test]
fn enu_matches_reference() {
    // the reference has no ENU; its ENU outputs are NED with north/east swapped and down negated
    for t in TARGET_REFS {
        let actual = enu2ned_axes(ecef2enu(t.ecef, t.lla_ref));
        assert_vec_near(
            actual,
            t.ned,
            POS_TOL_M,
            &format!("ecef2enu({:?}, {:?})", t.ecef, t.lla_ref),
        );
    }
    for row in LOCAL_VECTORS {
        let ctx = format!("{:?} at {:?}", row.local, row.lla_ref);
        assert_vec_near(
            enu2ecef(row.local, row.lla_ref),
            row.enu2ecef,
            POS_TOL_M,
            &format!("enu2ecef {ctx}"),
        );
        assert_vec_near(
            enu2ecef_uvw(row.local, row.lla_ref),
            row.enu2ecef_uvw,
            POS_TOL_M,
            &format!("enu2ecef_uvw {ctx}"),
        );
        assert_vec_near(
            enu2ned_axes(ecef_uvw2enu(row.local, row.lla_ref)),
            row.ecef_uvw2ned,
            POS_TOL_M,
            &format!("ecef_uvw2enu {ctx}"),
        );
    }
}

#[test]
fn lla_ned_composites_match_reference() {
    for t in TARGET_REFS {
        let ctx = format!("{:?} at {:?}", t.lla, t.lla_ref);
        assert_vec_near(
            ecef2ned(lla2ecef(t.lla), t.lla_ref),
            t.lla_ned,
            POS_TOL_M,
            &format!("lla2ned {ctx}"),
        );

        // ned2lla is checked against the target's true LLA, since the reference's ned2lla
        // inherits the NaN of its ecef2lla near the polar axis
        let at_pole = f64::abs(t.lla.x) == 90.0;
        assert_lla_near(
            ecef2lla(ned2ecef(t.lla_ned, t.lla_ref)),
            t.lla,
            LATLON_TOL_RAD,
            POS_TOL_M,
            !at_pole,
            &format!("ned2lla {ctx}"),
        );
    }
}

#[test]
fn local_aer_matches_reference() {
    let ang_tol_deg = LATLON_TOL_RAD.to_degrees();
    for row in NED2AER {
        let ned = row.ned;
        assert_aer_near(
            ned2aer(ned),
            row.aer,
            ang_tol_deg,
            POS_TOL_M,
            0.0,
            &format!("ned2aer({ned:?})"),
        );
        let enu = glam::DVec3::new(ned.y, ned.x, -ned.z);
        assert_aer_near(
            enu2aer(enu),
            row.aer,
            ang_tol_deg,
            POS_TOL_M,
            0.0,
            &format!("enu2aer({enu:?})"),
        );
    }
    for row in AER2NED {
        let aer = row.aer;
        assert_vec_near(
            aer2ned(aer),
            row.ned,
            POS_TOL_M,
            &format!("aer2ned({aer:?})"),
        );
        assert_vec_near(
            enu2ned_axes(aer2enu(aer)),
            row.ned,
            POS_TOL_M,
            &format!("aer2enu({aer:?})"),
        );
    }
}

#[test]
fn ecef_aer_matches_reference() {
    let ang_tol_deg = LATLON_TOL_RAD.to_degrees();
    for t in TARGET_REFS {
        let ctx = format!("{:?} at {:?}", t.ecef, t.lla_ref);
        assert_aer_near(
            ecef2aer(t.ecef, t.lla_ref),
            t.aer,
            ang_tol_deg,
            POS_TOL_M,
            POS_TOL_M,
            &format!("ecef2aer {ctx}"),
        );

        // rotating the vector rounds each component by about 1e-15 of its magnitude
        let rotation_noise_m = 1e-15 * t.ecef.length();
        assert_aer_near(
            ecef_uvw2aer(t.ecef, t.lla_ref),
            t.uvw_aer,
            ang_tol_deg,
            POS_TOL_M,
            rotation_noise_m,
            &format!("ecef_uvw2aer {ctx}"),
        );
    }
    for row in AER2ECEF {
        let ctx = format!("{:?} at {:?}", row.aer, row.lla_ref);
        assert_vec_near(
            aer2ecef(row.aer, row.lla_ref),
            row.ecef,
            POS_TOL_M,
            &format!("aer2ecef {ctx}"),
        );
        assert_vec_near(
            aer2ecef_uvw(row.aer, row.lla_ref),
            row.ecef_uvw,
            POS_TOL_M,
            &format!("aer2ecef_uvw {ctx}"),
        );
    }
}

#[test]
fn dcms_match_reference() {
    for row in DCM_BASIS {
        let ctx = format!("{:?} at {:?}", row.axis, row.lla_ref);
        assert_vec_near(
            ecef2ned_dcm(row.lla_ref) * row.axis,
            row.ecef2ned,
            1e-15,
            &format!("ecef2ned_dcm {ctx}"),
        );
        assert_vec_near(
            enu2ned_axes(ecef2enu_dcm(row.lla_ref) * row.axis),
            row.ecef2ned,
            1e-15,
            &format!("ecef2enu_dcm {ctx}"),
        );
        assert_vec_near(
            ned2ecef_dcm(row.lla_ref) * row.axis,
            row.ned2ecef,
            1e-15,
            &format!("ned2ecef_dcm {ctx}"),
        );
        assert_vec_near(
            enu2ecef_dcm(row.lla_ref) * row.axis,
            row.enu2ecef,
            1e-15,
            &format!("enu2ecef_dcm {ctx}"),
        );
    }
}

// --- Self-consistency (no reference needed) ---

#[test]
fn local_frames_round_trip() {
    for t in TARGET_REFS {
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
    for t in TARGET_REFS {
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
