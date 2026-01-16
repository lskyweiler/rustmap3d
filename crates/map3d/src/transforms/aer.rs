use crate::{
    transforms::enu,
    traits::{IntoDVec3, IntoLatLonTriple, IntoLatLonTuple},
};
use glam::{self, Vec3Swizzles};

/// Converts ENU to AER.
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

    let az = f64::atan2(enu.x, enu.y);
    let el = f64::atan2(enu.z, enu.xy().length());
    let aer = glam::DVec3::new(f64::to_degrees(az), f64::to_degrees(el), enu.length());
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

    let az = f64::atan2(ned.y, ned.x);
    let el = f64::atan2(-ned.z, ned.xy().length());
    let aer = glam::DVec3::new(f64::to_degrees(az), f64::to_degrees(el), ned.length());
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
    use crate::utils::assert_vecs_close;
    use rstest::*;

    #[rstest]
    fn test_enu2rae() {
        let actual_aer = enu2aer(&glam::DVec3::new(1000., 100., 10000.));
        let expected_aer =
            glam::DVec3::new(84.28940686250037, 84.26111457290625, 10050.373127401788);
        assert_vecs_close(&actual_aer, &expected_aer, 1e-6);
    }
    #[rstest]
    fn test_rae2enu() {
        let actual_enu = aer2enu(&glam::DVec3::new(15., 370., 123450.));
        let expected_enu =
            glam::DVec3::new(31465.800427043872, 117431.96589455023, 21436.8675329825);
        assert_vecs_close(&actual_enu, &expected_enu, 1e-6);
    }
}
