use crate::{
    traits::{IntoDVec3, IntoLatLonTriple, IntoLatLonTuple},
    transforms::enu,
    utils,
};
use glam::{self, Vec3Swizzles};

/// Converts ENU to AER.
///
/// Az Domain [0., 360.]
/// El Domain [-90., 90.]
///
/// # Arguments
///
/// * `enu` - Vector represented in ENU coordinates [[meters]].
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [[degrees-degrees-meters]].
pub fn enu2aer(enu: impl IntoDVec3) -> glam::DVec3 {
    let enu = enu.into_dvec3();

    let az = f64::atan2(enu.x, enu.y).to_degrees();
    let el = f64::atan2(enu.z, enu.xy().length());

    let az = utils::wrap_to_0_360(az);
    let el = utils::wrap_to_pi(el).to_degrees();

    let aer = glam::DVec3::new(az, el, enu.length());
    return aer;
}

/// Converts AER to ENU.
///
/// # Arguments
///
/// * `aer` - Vector represented in AER coordinates [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `enu` - Vector represented in ENU coordinates [[meters]].
pub fn aer2enu(aer: impl IntoDVec3) -> glam::DVec3 {
    let aer = aer.into_dvec3();

    let r = aer.z;
    let az = f64::to_radians(aer.x);
    let el = f64::to_radians(aer.y);
    let east_north = r * f64::cos(el);
    let east = east_north * f64::sin(az);
    let north = east_north * f64::cos(az);
    let up = r * f64::sin(el);
    return glam::DVec3::new(east, north, up);
}

/// Converts Absolute ECEF to AER.
/// This is a vector that is not in relation to an ECEF location
///
/// # Arguments
///
/// * `ecef` - Absolute ECEF position [m, m, m]
/// * `lla_ref` - Reference latitude-longitude-altitude [[radians-radians-meters]].
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [[degrees-degrees-meters]].
pub fn ecef2aer(ecef: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> glam::DVec3 {
    let enu = enu::ecef2enu(ecef, lla_ref);
    return enu2aer(&enu);
}
/// Converts ECEF uvw to AER.
/// This is a vector that is not an absolute ECEF location
///
/// # Arguments
///
/// * `ecef_uvw` - Vector represented in ECEF frame [[meters]].
/// * `ll_ref` - Reference latitude-longitude [[degrees-degrees]].
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [[degrees-degrees-meters]].
pub fn ecef_uvw2aer(ecef_uvw: impl IntoDVec3, ll_ref: impl IntoLatLonTuple) -> glam::DVec3 {
    let enu = enu::ecef_uvw2enu(ecef_uvw, ll_ref);
    return enu2aer(&enu);
}

/// Converts AER to ECEF uvw.
///
/// # Arguments
///
/// * `aer` - Vector represented in AER coordinates [[degrees-degrees-meters]].
/// * `ll_ref` - Reference latitude-longitude [[degrees-degrees]].
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
pub fn aer2ecef_uvw(aer: impl IntoDVec3, ll_ref: impl IntoLatLonTuple) -> glam::DVec3 {
    let enu = aer2enu(aer);
    return enu::enu2ecef_uvw(&enu, ll_ref);
}
/// Converts AER to Absolute ECEF.
///
/// # Arguments
///
/// * `aer` - Vector represented in AER coordinates [[degrees-degrees-meters]].
/// * `lla_ref` - Reference latitude-longitude-altitude [[radians-radians-meters]].
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
pub fn aer2ecef(aer: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> glam::DVec3 {
    let enu = aer2enu(aer);
    return enu::enu2ecef(&enu, lla_ref);
}

/// Converts NED to AER.
///
/// # Arguments
///
/// * `ned` - Vector represented in NED coordinates [[meters]].
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [[degrees-degrees-meters]].
pub fn ned2aer(ned: impl IntoDVec3) -> glam::DVec3 {
    let ned = ned.into_dvec3();

    let az = f64::atan2(ned.y, ned.x).to_degrees();
    let el = f64::atan2(-ned.z, ned.xy().length());

    let az = utils::wrap_to_0_360(az);
    let el = utils::wrap_to_pi(el).to_degrees();

    let aer = glam::DVec3::new(az, el, ned.length());
    return aer;
}

/// Converts AER to NED.
///
/// # Arguments
///
/// * `aer` - Vector represented in AER coordinates [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `ned` - Vector represented in NED coordinates [[meters]].
pub fn aer2ned(aer: impl IntoDVec3) -> glam::DVec3 {
    let enu = aer2enu(aer);
    return glam::DVec3::new(enu.y, enu.x, -enu.z);
}

#[cfg(test)]
mod test_aer {
    use super::*;
    use rstest::*;

    #[rstest]
    fn test_enu2rae() {
        let actual_aer = enu2aer(&glam::DVec3::new(1000., 100., 10000.));
        let expected_aer =
            glam::DVec3::new(84.28940686250037, 84.26111457290625, 10050.373127401788);

        assert!(actual_aer.abs_diff_eq(expected_aer, 1e-6));
    }
    #[rstest]
    fn test_rae2enu() {
        let actual_enu = aer2enu(&glam::DVec3::new(15., 370., 123450.));
        let expected_enu =
            glam::DVec3::new(31465.800427043872, 117431.96589455023, 21436.8675329825);
        assert!(actual_enu.abs_diff_eq(expected_enu, 1e-6));
    }

    #[rstest]
    #[case(glam::DVec3::new(0., 0., 5.), 90.)]
    #[case(glam::DVec3::new(0., 0., -5.), -90.)]
    #[case(glam::DVec3::new(3., 4., 0.), 0.)]
    fn test_enu2aer_elevation_extremes(#[case] enu: glam::DVec3, #[case] el: f64) {
        let aer = enu2aer(enu);
        assert_eq!(aer.y, el, "{aer:?}");
        assert_eq!(aer.z, enu.length());
    }

    #[test]
    fn test_enu2aer_zero_vector() {
        assert_eq!(enu2aer(glam::DVec3::ZERO), glam::DVec3::ZERO);
        assert_eq!(ned2aer(glam::DVec3::ZERO), glam::DVec3::ZERO);
    }

    #[test]
    fn test_aer_domains() {
        for e in [-1.0, -1e-9, 0.0, 1e-9, 1.0] {
            for n in [-1.0, -1e-9, 0.0, 1e-9, 1.0] {
                for u in [-1.0, 0.0, 1.0] {
                    let enu = glam::DVec3::new(e, n, u);
                    let aer = enu2aer(enu);
                    assert!(
                        (0.0..=360.0).contains(&aer.x),
                        "enu2aer az out of [0, 360]: {enu:?} -> {aer:?}"
                    );
                    assert!(
                        (-90.0..=90.0).contains(&aer.y),
                        "enu2aer el out of [-90, 90]: {enu:?} -> {aer:?}"
                    );
                    let ned = glam::DVec3::new(n, e, -u);
                    assert_eq!(
                        ned2aer(ned),
                        aer,
                        "ned2aer disagrees with enu2aer for {enu:?}"
                    );
                }
            }
        }
    }

    #[rstest]
    #[case(glam::DVec3::new(30., 40., 70.))]
    #[case(glam::DVec3::new(210., -10., 5.0e5))]
    #[case(glam::DVec3::new(359.5, 89., 1.))]
    #[case(glam::DVec3::new(0.25, -89., 1.0e7))]
    fn test_aer_round_trip(#[case] aer: glam::DVec3) {
        for (frame, actual) in [
            ("enu", enu2aer(aer2enu(aer))),
            ("ned", ned2aer(aer2ned(aer))),
        ] {
            let angles_ok = actual.xy().abs_diff_eq(aer.xy(), 1e-9);
            let range_ok = f64::abs(actual.z - aer.z) <= 1e-15 * aer.z;
            assert!(
                angles_ok && range_ok,
                "{frame}: {actual:?} expected {aer:?}"
            );
        }
    }
}
