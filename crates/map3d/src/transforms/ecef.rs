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
