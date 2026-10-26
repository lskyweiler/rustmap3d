use crate::{traits::IntoLatLonTuple, transforms::enu};
use glam;

/// Computes the clockwise angle from North of a local body-frame's x-axis
///
/// # Arguments
///
/// * `ecef_dcm` - Local body frame expressed as a dcm
/// * `ll_ref` - lat lon in [deg, deg]
///
/// # Returns
///
/// *  `heading` - Clockwise angle off true north [[degrees]] of the body's x axis
pub fn ecef_dcm2heading(local2ecef_dcm: &glam::DMat3, ll_ref: impl IntoLatLonTuple) -> f64 {
    let ecef2enu = enu::ecef2enu_dcm(ll_ref);
    let enu_dcm = ecef2enu * (*local2ecef_dcm);
    let forward = enu_dcm.x_axis;
    return enu::enu2heading(&forward);
}

/// Computes the clockwise angle from North of a local body-frame's x-axis
///
/// # Arguments
///
/// * `local2ecef_quat` - Local body frame expressed as a quaternion
/// * `ll_ref` - lat lon in [deg, deg]
///
/// # Returns
///
/// *  `heading` - Clockwise angle off true north [[degrees]] of the body's x axis
pub fn ecef_quat2heading(local2ecef_quat: &glam::DQuat, ll_ref: impl IntoLatLonTuple) -> f64 {
    let ecef_dcm = glam::DMat3::from_quat(*local2ecef_quat);
    return ecef_dcm2heading(&ecef_dcm, ll_ref);
}

#[cfg(test)]
mod test_ecef {
    use super::*;
    use rstest::*;

    /// A body frame whose x axis points `heading_deg` clockwise from north, with z up, at `ll`.
    fn body2ecef_dcm(heading_deg: f64, ll: (f64, f64)) -> glam::DMat3 {
        let (s, c) = f64::sin_cos(heading_deg.to_radians());
        let forward = glam::DVec3::new(s, c, 0.0);
        let left = glam::DVec3::new(-c, s, 0.0);
        let body2enu = glam::DMat3::from_cols(forward, left, glam::DVec3::Z);
        return enu::enu2ecef_dcm(ll) * body2enu;
    }

    #[rstest]
    #[case(0.0, (0.0, 0.0))]
    #[case(30.0, (45.0, -120.0))]
    #[case(135.0, (-33.0, 151.0))]
    #[case(-120.0, (89.0, 10.0))]
    #[case(179.0, (-60.0, -179.5))]
    fn test_heading_from_dcm_and_quat(#[case] heading_deg: f64, #[case] ll: (f64, f64)) {
        let dcm = body2ecef_dcm(heading_deg, ll);
        let from_dcm = ecef_dcm2heading(&dcm, ll);
        assert!(
            f64::abs(from_dcm - heading_deg) < 1e-9,
            "dcm heading {from_dcm}, expected {heading_deg}"
        );
        let from_quat = ecef_quat2heading(&glam::DQuat::from_mat3(&dcm), ll);
        assert!(
            f64::abs(from_quat - heading_deg) < 1e-9,
            "quat heading {from_quat}, expected {heading_deg}"
        );
    }
}
