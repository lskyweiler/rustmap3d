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
/// * `x_m` - ECEF x in meters [float]
/// * `y_m` - ECEF y in meters [float]
/// * `z_m` - ECEF z in meters [float]
///
/// # Returns
///
/// * `lla` - Vector represented in WGS84 LLA [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2lla(x_m: f64, y_m: f64, z_m: f64) -> Vec3Tup {
    let lla = map3d::ecef2lla(&glam::dvec3(x_m, y_m, z_m));
    return vec3_to_tuple(&lla);
}

/// Converts WGS84 LLA to ECEF.
///
/// # Arguments
///
/// * `lat_d` - WGS84 Lat in degrees [float]
/// * `lon_d` - WGS84 Lon in degrees [float]
/// * `alt_m` - WGS84 MSL alt in meters [float]
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
/// * `u_m` - ECEF u in meters [float]
/// * `v_m` - ECEF v in meters [float]
/// * `w_m` - ECEF w in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `enu` - Vector represented in ENU coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_uvw2enu(u_m: f64, v_m: f64, w_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let enu = map3d::ecef_uvw2enu(
        &glam::dvec3(u_m, v_m, w_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&enu);
}
/// Converts an ECEF position to ENU.
///
/// # Arguments
///
/// * `x_m` - ECEF x in meters [float]
/// * `y_m` - ECEF y in meters [float]
/// * `z_m` - ECEF z in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `enu` - Vector represented in ENU coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2enu(x_m: f64, y_m: f64, z_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let enu = map3d::ecef2enu(
        &glam::dvec3(x_m, y_m, z_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&enu);
}
/// Converts an absolute LLA position to an ENU vector relative to a lla reference
///
/// # Arguments
///
/// * `lat_d` - WGS84 Lat in degrees [float]
/// * `lon_d` - WGS84 Lon in degrees [float]
/// * `alt_m` - WGS84 MSL alt in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `enu` - Vector represented in ENU coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn lla2enu(lat_d: f64, lon_d: f64, alt_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef = map3d::lla2ecef(&glam::dvec3(lat_d, lon_d, alt_m));
    let enu = map3d::ecef2enu(&ecef, &glam::dvec3(lat_ref_d, lon_ref_d, 0.));
    return vec3_to_tuple(&enu);
}

/// Converts ENU to ECEF uvw vector
///
/// # Arguments
///
/// * `e_m` - East in meters [float]
/// * `n_m` - North in meters [float]
/// * `u_m` - Up in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `ecef_uvw` - Vector represented in ECEF frame. Not an absolute position [[meters]].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef_uvw(e_m: f64, n_m: f64, u_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef_uvw = map3d::enu2ecef_uvw(
        &glam::dvec3(e_m, n_m, u_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef_uvw);
}
/// Converts ENU to an ECEF position
///
/// # Arguments
///
/// * `e_m` - East in meters [float]
/// * `n_m` - North in meters [float]
/// * `u_m` - Up in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `ecef` - Absolute ECEF position in meters
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef(e_m: f64, n_m: f64, u_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef = map3d::enu2ecef(
        &glam::dvec3(e_m, n_m, u_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef);
}
/// Converts ENU to an absolute LLA position
///
/// # Arguments
///
/// * `e_m` - East in meters [float]
/// * `n_m` - North in meters [float]
/// * `u_m` - Up in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `lla` - Absolute LLA position [deg-deg-m]
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2lla(e_m: f64, n_m: f64, u_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef = map3d::enu2ecef(
        &glam::dvec3(e_m, n_m, u_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    let lla = map3d::ecef2lla(&ecef);
    return vec3_to_tuple(&lla);
}

/// Converts ECEF UVW Vector to NED.
///
/// # Arguments
///
/// * `u_m` - ECEF u in meters [float]
/// * `v_m` - ECEF v in meters [float]
/// * `w_m` - ECEF w in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `ned` - Vector represented in NED coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_uvw2ned(u_m: f64, v_m: f64, w_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ned = map3d::ecef_uvw2ned(
        &glam::dvec3(u_m, v_m, w_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ned);
}
/// Converts an absolute ECEF location to NED.
///
/// # Arguments
///
/// * `x_m` - ECEF x in meters [float]
/// * `y_m` - ECEF y in meters [float]
/// * `z_m` - ECEF z in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `ned` - Vector represented in NED coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2ned(x_m: f64, y_m: f64, z_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ned = map3d::ecef2ned(
        &glam::dvec3(x_m, y_m, z_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ned);
}
/// Converts an absolute LLA location to a NED vector relative to a lla reference point
///
/// # Arguments
///
/// * `lat_d` - WGS84 Lat in degrees [float]
/// * `lon_d` - WGS84 Lon in degrees [float]
/// * `alt_m` - WGS84 MSL alt in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `ned` - Vector represented in NED coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn lla2ned(lat_d: f64, lon_d: f64, alt_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef = map3d::lla2ecef(&glam::dvec3(lat_d, lon_d, alt_m));
    let ned = map3d::ecef2ned(&ecef, &glam::dvec3(lat_ref_d, lon_ref_d, 0.));
    return vec3_to_tuple(&ned);
}
/// Converts NED to ECEF uvw
///
/// # Arguments
///
/// * `n_m` - North in meters [float]
/// * `e_m` - East in meters [float]
/// * `d_m` - Down in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `ecef_uvw` - Vector represented in ECEF frame. Not an absolute position [[meters]].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef_uvw(n_m: f64, e_m: f64, d_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef_uvw = map3d::ned2ecef_uvw(
        &glam::dvec3(n_m, e_m, d_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef_uvw);
}
/// Converts NED to an absolute ECEF location
///
/// # Arguments
///
/// * `n_m` - North in meters [float]
/// * `e_m` - East in meters [float]
/// * `d_m` - Down in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `ecef` - Absolute ECEF position in meters
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef(n_m: f64, e_m: f64, d_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef = map3d::ned2ecef(
        &glam::dvec3(n_m, e_m, d_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef);
}
/// Converts a NED vector to an absolute lat lon alt
///
/// # Arguments
///
/// * `n_m` - North in meters [float]
/// * `e_m` - East in meters [float]
/// * `d_m` - Down in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `lla` - Absolute lat lon alt [deg-deg-m]
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2lla(n_m: f64, e_m: f64, d_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef = map3d::ned2ecef(
        &glam::dvec3(n_m, e_m, d_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    let lla = map3d::ecef2lla(&ecef);
    return vec3_to_tuple(&lla);
}

/// Converts ECEF uvw to AER (az el range).
///
/// # Arguments
///
/// * `u_m` - ECEF u in meters [float]
/// * `v_m` - ECEF v in meters [float]
/// * `w_m` - ECEF w in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_uvw2aer(u_m: f64, v_m: f64, w_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let aer = map3d::ecef_uvw2aer(
        &glam::dvec3(u_m, v_m, w_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&aer);
}
/// Converts an absolute ECEF to AER (az el range).
///
/// # Arguments
///
/// * `x_m` - ECEF x in meters [float]
/// * `y_m` - ECEF y in meters [float]
/// * `z_m` - ECEF z in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2aer(x_m: f64, y_m: f64, z_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let aer = map3d::ecef2aer(
        &glam::dvec3(x_m, y_m, z_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&aer);
}

/// Converts a vector direction and distance given in AER to ECEF uvw
///
/// # Arguments
///
/// * `a_d` - Azimuth in degrees [float]
/// * `e_d` - Elevation in degrees [float]
/// * `r_m` - Range in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `ecef_uvw` - Vector represented in ECEF frame. Not an absolute position [[meters]].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2ecef_uvw(a_d: f64, e_d: f64, r_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef_uvw = map3d::aer2ecef_uvw(
        &glam::dvec3(a_d, e_d, r_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef_uvw);
}
/// Converts AER from a given lat/lon to a new ECEF location
///
/// # Arguments
///
/// * `a_d` - Azimuth in degrees [float]
/// * `e_d` - Elevation in degrees [float]
/// * `r_m` - Range in meters [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * `ecef` - Absolute ECEF position in meters
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2ecef(a_d: f64, e_d: f64, r_m: f64, lat_ref_d: f64, lon_ref_d: f64) -> Vec3Tup {
    let ecef_uvw = map3d::aer2ecef(
        &glam::dvec3(a_d, e_d, r_m),
        &glam::dvec3(lat_ref_d, lon_ref_d, 0.),
    );
    return vec3_to_tuple(&ecef_uvw);
}

/// Converts ENU to AER.
///
/// # Arguments
///
/// * `e_m` - East in meters [float]
/// * `n_m` - North in meters [float]
/// * `u_m` - Up in meters [float]
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2aer(e_m: f64, n_m: f64, u_m: f64) -> Vec3Tup {
    let aer = map3d::enu2aer(&glam::dvec3(e_m, n_m, u_m));
    return vec3_to_tuple(&aer);
}

/// Converts AER vector direction and magnitude to ENU.
///
/// # Arguments
///
/// * `a_d` - Azimuth in degrees [float]
/// * `e_d` - Elevation in degrees [float]
/// * `r_m` - Range in meters [float]
///
/// # Returns
///
/// * `enu` - Vector represented in ENU coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2enu(a_d: f64, e_d: f64, r_m: f64) -> Vec3Tup {
    let enu = map3d::aer2enu(&glam::dvec3(a_d, e_d, r_m));
    return vec3_to_tuple(&enu);
}

/// Converts NED to AER.
///
/// # Arguments
///
/// * `n_m` - North in meters [float]
/// * `e_m` - East in meters [float]
/// * `d_m` - Down in meters [float]
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [degrees-degrees-meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2aer(n_m: f64, e_m: f64, d_m: f64) -> Vec3Tup {
    let aer = map3d::ned2aer(&glam::dvec3(n_m, e_m, d_m));
    return vec3_to_tuple(&aer);
}

/// Converts AER vector direction and magnitude to NED.
///
/// # Arguments
///
/// * `a_d` - Azimuth in degrees [float]
/// * `e_d` - Elevation in degrees [float]
/// * `r_m` - Range in meters [float]
///
/// # Returns
///
/// * `ned` - Vector represented in NED coordinates [meters].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2ned(a_d: f64, e_d: f64, r_m: f64) -> Vec3Tup {
    let ned = map3d::aer2enu(&glam::dvec3(a_d, e_d, r_m));
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
    let quat = map3d::utils::rand_orientation();
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an ECEF to ENU transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference in degrees [float].
/// * `lon_d` - Longitude reference in degrees [float].
///
/// # Returns
///
/// * `quat` - Normalized ECEF to ENU quaternion.
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2enu_quat(lat_d: f64, lon_d: f64) -> QuatTup {
    let quat = map3d::ecef2enu_quat((lat_d, lon_d));
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an ENU to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference in degrees [float].
/// * `lon_d` - Longitude reference in degrees [float].
///
/// # Returns
///
/// * `quat` - Normalized ENU to ECEF quaternion (w, x, y, z).
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef_quat(lat_d: f64, lon_d: f64) -> QuatTup {
    let quat = map3d::enu2ecef_quat((lat_d, lon_d));
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an ECEF to NED transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference in degrees [float].
/// * `lon_d` - Longitude reference in degrees [float].
///
/// # Returns
///
/// * `quat` - Normalized ECEF to NED quaternion (w, x, y, z).
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2ned_quat(lat_d: f64, lon_d: f64) -> QuatTup {
    let quat = map3d::ecef2ned_quat((lat_d, lon_d));
    return quat_to_tuple(&quat);
}

/// Calculates the quaternion that yields an NED to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference in degrees [float].
/// * `lon_d` - Longitude reference in degrees [float].
///
/// # Returns
///
/// * `quat` - Normalized NED to ECEF quaternion (w, x, y, z).
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef_quat(lat_d: f64, lon_d: f64) -> QuatTup {
    let quat = map3d::ned2ecef_quat((lat_d, lon_d));
    return quat_to_tuple(&quat);
}

/// Calculates the direction cosine matrix that yields an ECEF to ENU transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference in degrees [float].
/// * `lon_d` - Longitude reference in degrees [float].
///
/// # Returns
///
/// * `dcm` - ECEF to ENU direction cosine matrix [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]]
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2enu_dcm(lat_d: f64, lon_d: f64) -> Mat3Tup {
    let dcm = map3d::ecef2enu_dcm((lat_d, lon_d));
    return mat3_to_tuple(&dcm);
}

/// Calculates the direction cosine matrix that yields an ENU to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference in degrees [float].
/// * `lon_d` - Longitude reference in degrees [float].
///
/// # Returns
///
/// * `dcm` - ENU to ECEF direction cosine matrix [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]]
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef_dcm(lat_d: f64, lon_d: f64) -> Mat3Tup {
    let dcm = map3d::enu2ecef_dcm((lat_d, lon_d));
    return mat3_to_tuple(&dcm);
}

/// Calculates the direction cosine matrix that yields an ECEF to NED transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference in degrees [float].
/// * `lon_d` - Longitude reference in degrees [float].
///
/// # Returns
///
/// * `dcm` - ECEF to NED direction cosine matrix [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]]
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2ned_dcm(lat_d: f64, lon_d: f64) -> Mat3Tup {
    let dcm = map3d::ecef2ned_dcm((lat_d, lon_d));
    return mat3_to_tuple(&dcm);
}

/// Calculates the direction cosine matrix that yields an NED to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference in degrees [float].
/// * `lon_d` - Longitude reference in degrees [float].
///
/// # Returns
///
/// * `dcm` - NED to ECEF direction cosine matrix  [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]]
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef_dcm(lat_d: f64, lon_d: f64) -> Mat3Tup {
    let dcm = map3d::ned2ecef_dcm((lat_d, lon_d));
    return mat3_to_tuple(&dcm);
}

/// Calculates heading angle from ENU.
///
/// # Arguments
///
/// * `e_m` - East in meters [float]
/// * `n_m` - North in meters [float]
/// * `u_m` - Up in meters [float]
///
/// # Returns
///
/// * `heading_deg` - Heading angle relative to true north [degrees].
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2heading(e_m: f64, n_m: f64, u_m: f64) -> f64 {
    return map3d::enu2heading(&glam::dvec3(e_m, n_m, u_m));
}

/// Computes the clockwise angle from North of a local body-frame's x-axis
///
/// # Arguments
///
/// * `w` - Scalar part of quaternion [float]
/// * `x` - x part of quaternion [float]
/// * `y` - y part of quaternion [float]
/// * `z` - z part of quaternion [float]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
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
/// * `ax` - a vector x [float]
/// * `ay` - a vector y [float]
/// * `az` - a vector z [float]
/// * `bx` - b vector x [float]
/// * `by` - b vector y [float]
/// * `bz` - b vector z [float]
///
/// # Returns
///
/// * `angle` - angle between the two vectors in radians
#[gen_stub_pyfunction]
#[pyfunction]
pub fn angle_between(ax: f64, ay: f64, az: f64, bx: f64, by: f64, bz: f64) -> f64 {
    return map3d::utils::angle_between_vec3(&glam::dvec3(ax, ay, az), &glam::dvec3(bx, by, bz));
}

/// Calculates the LLA location that is a fixed range and bearing from a reference LLA. This function uses an iterative
/// solution to determine outputs using the WGS84 ellipsoidal Earth model.
///
/// See reference:
/// https://en.wikipedia.org/wiki/Vincenty%27s_formulae.
///
/// # Arguments
///
/// * `lat_d` - Latitude reference in degrees [float].
/// * `lon_d` - Longitude reference in degrees [float].
/// * `range_m` - Range (i.e., distance) from point A to point B [meters].
/// * `bearing_d` - Bearing (i.e., azimuth) from point A to point B relative to true north [degrees].
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
#[pyo3(signature = (lat_d, lon_d, range_m, bearing_d, atol=1.0E-13, max_iters=1000))]
pub fn vincenty_direct(
    lat_d: f64,
    lon_d: f64,
    range_m: f64,
    bearing_d: f64,
    atol: f64,
    max_iters: u16,
) -> PyResult<(f64, f64)> {
    let result = map3d::vincenty_direct(lat_d, lon_d, range_m, bearing_d, atol, max_iters);

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
#[pyo3(signature = (lat_a_deg, lon_a_deg, lat_b_deg, lon_b_deg, atol=1.0E-13, max_iters=1000))]
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

/// Converts degrees-minutes-seconds (DMS) to decimal degrees (DD).
///
/// # Arguments
///
/// * `dms` - Format "{Degrees}:{Minutes}:{Seconds}.{DecSeconds}{Direction}" where Direction is one of N, E, S, or W.
///
/// # Returns
///
/// * `dd` - Decimal degrees [degrees].
///
/// # Examples
/// ```
/// dd = map3d.dms2dd("25:22:44.738N")  #> 25.37909389
/// ```
#[gen_stub_pyfunction]
#[pyfunction]
pub fn dms2dd(dms: String) -> PyResult<f64> {
    match map3d::dms2dd(&dms) {
        Ok(dd) => Ok(dd),
        Err(e) => Err(PyValueError::new_err(e.to_string())),
    }
}
/// Converts decimal degrees (DD) to degrees-minutes-seconds (DMS).
///
/// # Arguments
///
/// * `dd` - Decimal degrees [degrees].
/// * `is_lat` - Flag to denote if this decimal value describes a latitude [bool]
///
/// # Returns
///
/// * `dms` - Format "{Degrees}:{Minutes}:{Seconds}.{DecSeconds}{Direction}" where Direction is one of N, E, S, or W.
///
/// # Examples
/// ```
/// dms = map3d.dd2dms(25.37909389, true) #> "25:22:44.738N"
/// ```
#[gen_stub_pyfunction]
#[pyfunction]
pub fn dd2dms(dd: f64, is_lat: bool) -> String {
    return map3d::dd2dms(dd, is_lat);
}

/// Convenience function to convert lat/lon to a tuple of (lat dms, lon dms)
///
/// # Arguments
///
/// * `lat` - Latitude in decimal degrees
/// * `lon` - Longitude in decimal degrees
///
/// # Returns
///
/// * `(lat dms, lon dms)` - Tuple of lat/lon as degrees:minutes:seconds [Tuple[String, String]]
/// ```
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ll2dms(lat_d: f64, lon_d: f64) -> (String, String) {
    return map3d::ll2dms((lat_d, lon_d));
}
