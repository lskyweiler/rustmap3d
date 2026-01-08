use map3d;
use pyo3::{exceptions::PyValueError, prelude::*};
use pyo3_stub_gen::derive::*;

/// Vector as a tuple [x, y, z]
pub type Vec3Tup = (f64, f64, f64);

/// Quaternion as a tuple [w, x, y, w]
pub type QuatTup = (f64, f64, f64, f64);

/// Column major 3x3 matrix [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]]
pub type Mat3Tup = (f64, f64, f64, f64, f64, f64, f64, f64, f64);

fn vec3_to_tuple(vec3: &glam::DVec3) -> Vec3Tup {
    return (vec3.x, vec3.y, vec3.z);
}

fn quat_to_tuple(q: &glam::DQuat) -> QuatTup {
    return (q.w, q.x, q.y, q.z);
}

fn mat3_to_tuple(mat3: &glam::DMat3) -> Mat3Tup {
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

/// Converts ECEF to WGS84 LLA
///
/// # Arguments
///
/// * `x` - ECEF x in meters [f64]
/// * `y` - ECEF y in meters [f64]
/// * `z` - ECEF z in meters [f64]
///
/// # Returns
///
/// * `lla` - Vector represented in WGS84 LLA [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2lla(x: f64, y: f64, z: f64) -> Vec3Tup {
    let lla = map3d::ecef2lla(&glam::dvec3(x, y, z));
    return vec3_to_tuple(&lla);
}

/// Converts WGS84 LLA to ECEF.
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

/// Converts an ECEF uvw vector to ENU.
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
/// * `enu` - Vector represented in ENU coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_uvw2enu(u: f64, v: f64, w: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let enu = map3d::ecef_uvw2enu(
        &glam::dvec3(u, v, w),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&enu);
}
/// Converts an ECEF position to ENU.
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
pub fn ecef2enu(x: f64, y: f64, z: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let enu = map3d::ecef2enu(
        &glam::dvec3(x, y, z),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&enu);
}

/// Converts ENU to ECEF uvw vector
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
/// * `ecef_uvw` - Vector represented in ECEF frame. Not an absolute position [[meters]].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef_uvw(e: f64, n: f64, u: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef_uvw = map3d::enu2ecef_uvw(
        &glam::dvec3(e, n, u),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef_uvw);
}
/// Converts ENU to an ECEF position
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
/// * `ecef` - Absolute ECEF position in meters
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef(e: f64, n: f64, u: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef = map3d::enu2ecef(
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
/// Converts an absolute ECEF location to NED.
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
pub fn ecef2ned(x: f64, y: f64, z: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ned = map3d::ecef2ned(
        &glam::dvec3(x, y, z),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ned);
}
/// Converts NED to ECEF uvw
///
/// # Arguments
///
/// * `n` - North in meters [f64]
/// * `e` - East in meters [f64]
/// * `d` - Down in meters [f64]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees
/// * `lon_ref_d` - Reference WGS84 Lon in degrees
///
/// # Returns
///
/// * `ecef_uvw` - Vector represented in ECEF frame. Not an absolute position [[meters]].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef_uvw(n: f64, e: f64, d: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef_uvw = map3d::ned2ecef_uvw(
        &glam::dvec3(n, e, d),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef_uvw);
}
/// Converts NED to an absolute ECEF location
///
/// # Arguments
///
/// * `n` - North in meters [f64]
/// * `e` - East in meters [f64]
/// * `d` - Down in meters [f64]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees
/// * `lon_ref_d` - Reference WGS84 Lon in degrees
///
/// # Returns
///
/// * `ecef` - Absolute ECEF position in meters
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef(n: f64, e: f64, d: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef = map3d::ned2ecef(
        &glam::dvec3(n, e, d),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef);
}

/// Converts ECEF uvw to AER.
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
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_uvw2aer(u: f64, v: f64, w: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let aer = map3d::ecef_uvw2aer(
        &glam::dvec3(u, v, w),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&aer);
}
/// Converts an absolute ECEF to AER.
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
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2aer(x: f64, y: f64, z: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let aer = map3d::ecef2aer(
        &glam::dvec3(x, y, z),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&aer);
}

/// Converts AER to ECEF uvw
///
/// # Arguments
///
/// * `a` - Azimuth in degrees [f64]
/// * `e` - Elevation in degrees [f64]
/// * `r` - Range in meters [f64]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees
/// * `lon_ref_d` - Reference WGS84 Lon in degrees
///
/// # Returns
///
/// * `ecef_uvw` - Vector represented in ECEF frame. Not an absolute position [[meters]].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2ecef_uvw(a: f64, e: f64, r: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef_uvw = map3d::aer2ecef_uvw(
        &glam::dvec3(a, e, r),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef_uvw);
}
/// Converts AER to an ECEF location
///
/// # Arguments
///
/// * `a` - Azimuth in degrees [f64]
/// * `e` - Elevation in degrees [f64]
/// * `r` - Range in meters [f64]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees
/// * `lon_ref_d` - Reference WGS84 Lon in degrees
///
/// # Returns
///
/// * `ecef` - Absolute ECEF position in meters
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2ecef(a: f64, e: f64, r: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef_uvw = map3d::aer2ecef(
        &glam::dvec3(a, e, r),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef_uvw);
}

/// Converts ENU to AER.
///
/// # Arguments
///
/// * `e` - East in meters [f64]
/// * `n` - North in meters [f64]
/// * `u` - Up in meters [f64]
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2aer(e: f64, n: f64, u: f64) -> Vec3Tup {
    let aer = map3d::enu2aer(&glam::dvec3(e, n, u));
    return vec3_to_tuple(&aer);
}

/// Converts AER to ENU.
///
/// # Arguments
///
/// * `a` - Azimuth in degrees [f64]
/// * `e` - Elevation in degrees [f64]
/// * `r` - Range in meters [f64]
///
/// # Returns
///
/// * `enu` - Vector represented in ENU coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2enu(a: f64, e: f64, r: f64) -> Vec3Tup {
    let enu = map3d::aer2enu(&glam::dvec3(a, e, r));
    return vec3_to_tuple(&enu);
}

/// Converts NED to AER.
///
/// # Arguments
///
/// * `n` - North in meters [f64]
/// * `e` - East in meters [f64]
/// * `d` - Down in meters [f64]
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2aer(n: f64, e: f64, d: f64) -> Vec3Tup {
    let aer = map3d::ned2aer(&glam::dvec3(n, e, d));
    return vec3_to_tuple(&aer);
}

/// Converts AER to NED.
///
/// # Arguments
///
/// * `a` - Azimuth in degrees [f64]
/// * `e` - Elevation in degrees [f64]
/// * `r` - Range in meters [f64]
///
/// # Returns
///
/// * `ned` - Vector represented in NED coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2ned(a: f64, e: f64, r: f64) -> Vec3Tup {
    let ned = map3d::aer2enu(&glam::dvec3(a, e, r));
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
/// * `quat` - Random normalized quaternion (w, x, y, z).
#[gen_stub_pyfunction]
#[pyfunction]
pub fn rand_orientation() -> QuatTup {
    let quat = map3d::util::rand_orientation();
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an ECEF to ENU transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference [degrees].
/// * `lon_d` - Longitude reference [degrees].
///
/// # Returns
///
/// * `quat` - Normalized ECEF to ENU quaternion.
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2enu_quat(lat_d: f64, lon_d: f64) -> QuatTup {
    let quat = map3d::ecef2enu_quat(lat_d, lon_d);
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an ENU to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference [degrees].
/// * `lon_d` - Longitude reference [degrees].
///
/// # Returns
///
/// * `quat` - Normalized ENU to ECEF quaternion (w, x, y, z).
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef_quat(lat_d: f64, lon_d: f64) -> QuatTup {
    let quat = map3d::enu2ecef_quat(lat_d, lon_d);
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an ECEF to NED transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference [degrees].
/// * `lon_d` - Longitude reference [degrees].
///
/// # Returns
///
/// * `quat` - Normalized ECEF to NED quaternion (w, x, y, z).
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2ned_quat(lat_d: f64, lon_d: f64) -> QuatTup {
    let quat = map3d::ecef2ned_quat(lat_d, lon_d);
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an NED to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference [degrees].
/// * `lon_d` - Longitude reference [degrees].
///
/// # Returns
///
/// * `quat` - Normalized NED to ECEF quaternion (w, x, y, z).
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef_quat(lat_d: f64, lon_d: f64) -> QuatTup {
    let quat = map3d::ned2ecef_quat(lat_d, lon_d);
    return quat_to_tuple(&quat);
}

/// Calculates the direction cosine matrix that yields an ECEF to ENU transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference [degrees].
/// * `lon_d` - Longitude reference [degrees].
///
/// # Returns
///
/// * `dcm` - ECEF to ENU direction cosine matrix [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]]
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2enu_dcm(lat_d: f64, lon_d: f64) -> Mat3Tup {
    let dcm = map3d::ecef2enu_dcm(lat_d, lon_d);
    return mat3_to_tuple(&dcm);
}

/// Calculates the direction cosine matrix that yields an ENU to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference [degrees].
/// * `lon_d` - Longitude reference [degrees].
///
/// # Returns
///
/// * `dcm` - ENU to ECEF direction cosine matrix [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]]
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef_dcm(lat_d: f64, lon_d: f64) -> Mat3Tup {
    let dcm = map3d::enu2ecef_dcm(lat_d, lon_d);
    return mat3_to_tuple(&dcm);
}

/// Calculates the direction cosine matrix that yields an ECEF to NED transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference [degrees].
/// * `lon_d` - Longitude reference [degrees].
///
/// # Returns
///
/// * `dcm` - ECEF to NED direction cosine matrix [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]]
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2ned_dcm(lat_d: f64, lon_d: f64) -> Mat3Tup {
    let dcm = map3d::ecef2ned_dcm(lat_d, lon_d);
    return mat3_to_tuple(&dcm);
}

/// Calculates the direction cosine matrix that yields an NED to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference [degrees].
/// * `lon_d` - Longitude reference [degrees].
///
/// # Returns
///
/// * `dcm` - NED to ECEF direction cosine matrix  [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]]
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef_dcm(lat_d: f64, lon_d: f64) -> Mat3Tup {
    let dcm = map3d::ned2ecef_dcm(lat_d, lon_d);
    return mat3_to_tuple(&dcm);
}

/// Calculates heading angle from ENU.
///
/// # Arguments
///
/// * `e` - East in meters [f64]
/// * `n` - North in meters [f64]
/// * `u` - Up in meters [f64]
///
/// # Returns
///
/// * `heading_deg` - Heading angle relative to true north [degrees].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2heading(e: f64, n: f64, u: f64) -> f64 {
    return map3d::enu2heading(&glam::dvec3(e, n, u));
}

/// Computes the clockwise angle from North of a local body-frame's x-axis
///
/// # Arguments
///
/// * `w` - Scalar part of quaternion
/// * `x` - x part of quaternion
/// * `y` - y part of quaternion
/// * `z` - z part of quaternion
/// * `lat_ref_d` - Reference WGS84 Lat in degrees
/// * `lon_ref_d` - Reference WGS84 Lon in degrees
///
/// # Returns
///
/// *  `heading` - Clockwise angle off true north [[degrees]] of the body's x axis
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_quat2heading(w: f64, x: f64, y: f64, z: f64, lat_ref_d: f64, lon_ref_d: f64) -> f64 {
    let heading = map3d::ecef_quat2heading(
        &glam::dquat(x, y, z, w),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return heading;
}

/// Computes the angle between two vectors in radians. Dot product is clamped to [-1, 1] so this always outputs a number
///
/// # Arguments
///
/// * `ax` - a vector x
/// * `ay` - a vector y
/// * `az` - a vector z
/// * `bx` - b vector x
/// * `by` - b vector y
/// * `bz` - b vector z
///
/// # Returns
///
/// * `angle` - angle between the two vectors in radians
#[gen_stub_pyfunction]
#[pyfunction]
pub fn angle_between(ax: f64, ay: f64, az: f64, bx: f64, by: f64, bz: f64) -> f64 {
    return map3d::util::angle_between_vec3(&glam::dvec3(ax, ay, az), &glam::dvec3(bx, by, bz));
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
    lat_d: f64,
    long_d: f64,
    range_m: f64,
    bearing_deg: f64,
    atol: f64,
    max_iters: u16,
) -> PyResult<(f64, f64)> {
    let result = map3d::vincenty_direct(lat_d, long_d, range_m, bearing_deg, atol, max_iters);

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
