use map3d;
use pyo3::{exceptions::PyValueError, prelude::*};
use pyo3_stub_gen::derive::*;

pub type Vec3Tup = (f64, f64, f64);
pub type QuatTup = (f64, f64, f64, f64);
pub type Mat3Tup = (f64, f64, f64, f64, f64, f64, f64, f64, f64);

pub fn vec3_to_tuple(vec3: &glam::DVec3) -> Vec3Tup {
    return (vec3.x, vec3.y, vec3.z);
}

pub fn tuple_to_vec3(tup: &Vec3Tup) -> glam::DVec3 {
    return glam::DVec3::new(tup.0, tup.1, tup.2);
}

pub fn quat_to_tuple(q: &glam::DQuat) -> QuatTup {
    return (q.w, q.x, q.y, q.z);
}

pub fn tuple_to_quat(q: &QuatTup) -> glam::DQuat {
    return glam::DQuat::from_xyzw(q.1, q.2, q.3, q.0);
}

pub fn mat3_to_tuple(mat3: &glam::DMat3) -> Mat3Tup {
    return (
        mat3.x_axis.x,
        mat3.x_axis.y,
        mat3.x_axis.z,
        mat3.y_axis.x,
        mat3.y_axis.y,
        mat3.y_axis.z,
        mat3.z_axis.x,
        mat3.z_axis.y,
        mat3.z_axis.z,
    );
}

/// Converts ECEF to LLA.
///
/// # Arguments
///
/// * `x` - ECEF x in meters [f64]
/// * `y` - ECEF y in meters [f64]
/// * `z` - ECEF z in meters [f64]
///
/// # Returns
///
/// * `lla` - Vector represented in LLA coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2lla(x: f64, y: f64, z: f64) -> Vec3Tup {
    let lla = map3d::ecef2lla(&glam::dvec3(x, y, z));
    return vec3_to_tuple(&lla);
}

/// Converts LLA to ECEF.
///
/// # Arguments
///
/// * `lat_d` - WGS84 Lat in degrees
/// * `lon_d` - WGS84 Lon in degrees
/// * `alt_m` - WGS84 MSL alt in meters
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn lla2ecef(lat_d: f64, lon_d: f64, alt_m: f64) -> Vec3Tup {
    let ecef = map3d::lla2ecef(&glam::dvec3(lat_d, lon_d, alt_m));
    return vec3_to_tuple(&ecef);
}

/// Converts ECEF to ENU.
///
/// # Arguments
///
/// * `x` - ECEF x in meters [f64]
/// * `y` - ECEF y in meters [f64]
/// * `z` - ECEF z in meters [f64]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees
/// * `lon_ref_d` - Reference WGS84 Lon in degrees
///
/// # Returns
///
/// * `enu` - Vector represented in ENU coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_uvw2enu(x: f64, y: f64, z: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let enu = map3d::ecef_uvw2enu(
        &glam::dvec3(x, y, z),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&enu);
}

/// Converts ENU to ECEF.
///
/// # Arguments
///
/// * `e` - East in meters [f64]
/// * `n` - North in meters [f64]
/// * `u` - Up in meters [f64]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees
/// * `lon_ref_d` - Reference WGS84 Lon in degrees
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef_uvw(e: f64, n: f64, u: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef = map3d::enu2ecef_uvw(
        &glam::dvec3(e, n, u),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef);
}

/// Converts ECEF UVW Vector to NED.
///
/// # Arguments
///
/// * `u` - ECEF u in meters [f64]
/// * `v` - ECEF v in meters [f64]
/// * `w` - ECEF w in meters [f64]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees
/// * `lon_ref_d` - Reference WGS84 Lon in degrees
///
/// # Returns
///
/// * `ned` - Vector represented in NED coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_uvw2ned(u: f64, v: f64, w: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ned = map3d::ecef_uvw2ned(
        &glam::dvec3(u, v, w),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ned);
}

/// Converts NED to ECEF.
///
/// # Arguments
///
/// * `ned` - Vector represented in NED coordinates [meters].
/// * `lla_ref` - Reference latitude-longitude-altitude [degrees-degrees-meters].
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef_uvw(ned: Vec3Tup, lla_ref: Vec3Tup) -> Vec3Tup {
    let ecef = map3d::ned2ecef_uvw(&tuple_to_vec3(&ned), &tuple_to_vec3(&lla_ref));
    return vec3_to_tuple(&ecef);
}

/// Converts ECEF to AER.
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [meters].
/// * `ref_lla` - Reference latitude-longitude-altitude [radians-radians-meters].
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_uvw2aer(ecef: Vec3Tup, lla_ref: Vec3Tup) -> Vec3Tup {
    let aer = map3d::ecef_uvw2aer(&tuple_to_vec3(&ecef), &tuple_to_vec3(&lla_ref));
    return vec3_to_tuple(&aer);
}

/// Converts AER to ECEF.
///
/// # Arguments
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
/// * `ref_lla` - Reference latitude-longitude-altitude [radians-radians-meters].
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2ecef_uvw(aer: Vec3Tup, lla_ref: Vec3Tup) -> Vec3Tup {
    let ecef_uvw = map3d::aer2ecef_uvw(&tuple_to_vec3(&aer), &tuple_to_vec3(&lla_ref));
    return vec3_to_tuple(&ecef_uvw);
}

/// Converts ENU to AER.
///
/// # Arguments
///
/// * `enu` - Vector represented in ENU coordinates [meters].
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2aer(enu: Vec3Tup) -> Vec3Tup {
    let aer = map3d::enu2aer(&tuple_to_vec3(&enu));
    return vec3_to_tuple(&aer);
}

/// Converts AER to ENU.
///
/// # Arguments
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
///
/// # Returns
///
/// * `enu` - Vector represented in ENU coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2enu(aer: Vec3Tup) -> Vec3Tup {
    let enu = map3d::aer2enu(&tuple_to_vec3(&aer));
    return vec3_to_tuple(&enu);
}

/// Converts NED to AER.
///
/// # Arguments
///
/// * `ned` - Vector represented in NED coordinates [meters].
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2aer(ned: Vec3Tup) -> Vec3Tup {
    let aer = map3d::ned2aer(&tuple_to_vec3(&ned));
    return vec3_to_tuple(&aer);
}

/// Converts AER to NED.
///
/// # Arguments
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
///
/// # Returns
///
/// * `ned` - Vector represented in NED coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2ned(aer: Vec3Tup) -> Vec3Tup {
    let ned = map3d::aer2enu(&tuple_to_vec3(&aer));
    return vec3_to_tuple(&ned);
}

/// Generates a uniform random LLA point. Altitude is generated in the domain [0.0, 10000.0].
///
/// # Returns
///
/// * `lla` - Random LLA location [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn rand_lla() -> Vec3Tup {
    let lla = map3d::rand_lla();
    return vec3_to_tuple(&lla);
}

/// Generates a uniform random ECEF point on the surface of a spherical Earth.
///
/// # Returns
///
/// * `ecef` - Random ECEF location [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn rand_ecef() -> Vec3Tup {
    let ecef = map3d::rand_ecef();
    return vec3_to_tuple(&ecef);
}

/// Generates a random normalized quaternion.
///
/// # Returns
///
/// * `quat` - Random normalized quaternion.
#[gen_stub_pyfunction]
#[pyfunction]
pub fn rand_orientation() -> QuatTup {
    let quat = map3d::rand_orientation();
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an ECEF to ENU transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [degrees].
/// * `lon_deg` - Longitude reference [degrees].
///
/// # Returns
///
/// * `quat` - Normalized ECEF to ENU quaternion.
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2enu_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::ecef2enu_quat(lat, lon);
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an ENU to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [degrees].
/// * `lon_deg` - Longitude reference [degrees].
///
/// # Returns
///
/// * `quat` - Normalized ENU to ECEF quaternion.
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::enu2ecef_quat(lat, lon);
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an ECEF to NED transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [degrees].
/// * `lon_deg` - Longitude reference [degrees].
///
/// # Returns
///
/// * `quat` - Normalized ECEF to NED quaternion.
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2ned_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::ecef2ned_quat(lat, lon);
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an NED to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [degrees].
/// * `lon_deg` - Longitude reference [degrees].
///
/// # Returns
///
/// * `quat` - Normalized NED to ECEF quaternion.
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::ned2ecef_quat(lat, lon);
    return quat_to_tuple(&quat);
}

/// Calculates the direction cosine matrix that yields an ECEF to ENU transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [degrees].
/// * `lon_deg` - Longitude reference [degrees].
///
/// # Returns
///
/// * `dcm` - ECEF to ENU direction cosine matrix.
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2enu_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::ecef2enu_dcm(lat, lon);
    return mat3_to_tuple(&dcm);
}

/// Calculates the direction cosine matrix that yields an ENU to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [degrees].
/// * `lon_deg` - Longitude reference [degrees].
///
/// # Returns
///
/// * `dcm` - ENU to ECEF direction cosine matrix.
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::enu2ecef_dcm(lat, lon);
    return mat3_to_tuple(&dcm);
}

/// Calculates the direction cosine matrix that yields an ECEF to NED transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [degrees].
/// * `lon_deg` - Longitude reference [degrees].
///
/// # Returns
///
/// * `dcm` - ECEF to NED direction cosine matrix.
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2ned_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::ecef2ned_dcm(lat, lon);
    return mat3_to_tuple(&dcm);
}

/// Calculates the direction cosine matrix that yields an NED to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [degrees].
/// * `lon_deg` - Longitude reference [degrees].
///
/// # Returns
///
/// * `dcm` - NED to ECEF direction cosine matrix.
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::ned2ecef_dcm(lat, lon);
    return mat3_to_tuple(&dcm);
}

/// Calculates heading angle from ENU.
///
/// # Arguments
///
/// * `enu` - Vector represented in ENU coordinates [meters].
///
/// # Returns
///
/// * `heading_deg` - Heading angle relative to true north [degrees].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2heading(enu: Vec3Tup) -> f64 {
    return map3d::enu2heading(&tuple_to_vec3(&enu));
}

#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_quat2heading(ecef_quat: QuatTup, lla_ref: Vec3Tup) -> f64 {
    let heading = map3d::ecef_quat2heading(&tuple_to_quat(&ecef_quat), &tuple_to_vec3(&lla_ref));
    return heading;
}

#[gen_stub_pyfunction]
#[pyfunction]
pub fn angle_between(a: Vec3Tup, b: Vec3Tup) -> f64 {
    return map3d::angle_between_vec3(&tuple_to_vec3(&a), &tuple_to_vec3(&b));
}

/// Calculates the LLA location that is a fixed range and bearing from a reference LLA. This function uses an iterative
/// solution to determine outputs using the WGS84 ellipsoidal Earth model.
///
/// See reference:
/// https://en.wikipedia.org/wiki/Vincenty%27s_formulae.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [degrees].
/// * `lon_deg` - Longitude reference [degrees].
/// * `range_m` - Range (i.e., distance) from point A to point B [meters].
/// * `bearing_deg` - Bearing (i.e., azimuth) from point A to point B relative to true north [degrees].
/// * `abs_tol` - Absolute tolerance used for convergence.
/// * `max_iters` - Maximum possible number of iterations before early termination.
///
/// # Returns
///
/// A tuple `(lat_deg, lon_deg)` where:
/// * `lat_deg` - Latitude location [degrees].
/// * `lon_deg` - Longitude location [degrees].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn vincenty_direct(
    lat_deg: f64,
    lon_deg: f64,
    range_m: f64,
    bearing_deg: f64,
    atol: f64,
    max_iters: u16,
) -> PyResult<(f64, f64)> {
    let result = map3d::vincenty_direct(lat_deg, lon_deg, range_m, bearing_deg, atol, max_iters);

    match result {
        Ok(val) => Ok(val),
        Err(err) => Err(PyValueError::new_err(err.to_string())),
    }
}

/// Calculates range and bearings between two latitude-longitude points. This function uses an iterative solution to
/// determine outputs using the WGS84 ellipsoidal Earth model.
///
/// See reference:
/// https://en.wikipedia.org/wiki/Vincenty%27s_formulae.
///
/// # Arguments
///
/// * `lat_a_deg` - Latitude point A [degrees].
/// * `lon_a_deg` - Longitude point A [degrees].
/// * `lat_b_deg` - Latitude point A [degrees].
/// * `lon_b_deg` - Longitude point A [degrees].
/// * `atol` - Absolute tolerance used for convergence.
/// * `max_iters` - Maximum possible number of iterations before early termination.
///
/// # Returns
///
/// A tuple `(range_m, bearing_ab_deg, bearing_ba_deg)` where:
/// * `range_m` - Range (i.e., distance) from point A to point B [meters].
/// * `bearing_ab_deg` - Bearing (i.e., azimuth) from point A to point B relative to true north [degrees].
/// * `bearing_ba_deg` - Bearing (i.e., azimuth) from point B to point A relative to true north [degrees].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn vincenty_inverse(
    lat_a_deg: f64,
    lon_a_deg: f64,
    lat_b_deg: f64,
    lon_b_deg: f64,
    atol: f64,
    max_iters: u16,
) -> PyResult<(f64, f64, f64)> {
    let result =
        map3d::vincenty_inverse(lat_a_deg, lon_a_deg, lat_b_deg, lon_b_deg, atol, max_iters);

    match result {
        Ok(val) => Ok(val),
        Err(err) => Err(PyValueError::new_err(err.to_string())),
    }
}
