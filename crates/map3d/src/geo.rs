use core::fmt;
use std::f64::consts::PI;
use std::f64::INFINITY;

use crate::util;
use almost;
use chrono::{Datelike, NaiveDateTime, Timelike};
use glam::{self, Vec3Swizzles};
use rand;
use rand_distr::{Distribution, Normal};

pub mod geo_const {
    pub static EARTH_SEMI_MAJOR_AXIS: f64 = 6378137.0; // Equatorial radius.
    pub static EARTH_SEMI_MAJOR_AXIS_2: f64 = EARTH_SEMI_MAJOR_AXIS * EARTH_SEMI_MAJOR_AXIS;
    pub static EARTH_SEMI_MINOR_AXIS: f64 = 6356752.314245; // Polar radius.
    pub static EARTH_SEMI_MINOR_AXIS_2: f64 = EARTH_SEMI_MINOR_AXIS * EARTH_SEMI_MINOR_AXIS;
    pub static EARTH_FLATTENING_FACTOR: f64 = 0.003352810664740;
    pub static EARTH_ANGULAR_VEL_RADPS: f64 = 7.292115900000000e-05;
    pub static EARTH_E: f64 = 521854.0084255785;
    pub static EARTH_E_2: f64 = EARTH_E * EARTH_E;

    pub static ECEF2LLA_A: f64 = EARTH_SEMI_MAJOR_AXIS;
    pub static ECEF2LLA_B: f64 = ECEF2LLA_A * (1.0 - EARTH_FLATTENING_FACTOR);
    pub static ECEF2LLA_A2: f64 = ECEF2LLA_A * ECEF2LLA_A;
    pub static ECEF2LLA_B2: f64 = ECEF2LLA_B * ECEF2LLA_B;
    pub static ECEF2LLA_E2: f64 = 1.0 - ECEF2LLA_B2 / ECEF2LLA_A2;
    pub static ECEF2LLA_EP2: f64 = ECEF2LLA_A2 / ECEF2LLA_B2 - 1.0;
    pub static ECEF2LLA_EP22: f64 = ECEF2LLA_EP2 * ECEF2LLA_EP2;
}

mod vincenty_const {
    pub static INVERSE_SIN_SIGMA_TOL: f64 = 1.0e-10;
    pub static INVERSE_COS2A_TOL: f64 = 1.0e-10;
}

/// Converts ECEF to LLA using Ferrari's solution:
/// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#Ferrari's_solution
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
///
/// # Returns
///
/// * `lla` - Vector represented in LLA coordinates [[degrees-degrees-meters]].
pub fn ecef2lla_ferarri(ecef: &glam::DVec3) -> glam::DVec3 {
    let z_ecef_squared: f64 = f64::powf(ecef.z, 2.);
    let range_squared: f64 = f64::powf(ecef.x, 2.) + f64::powf(ecef.y, 2.);
    let range_: f64 = f64::sqrt(range_squared);
    let f_term: f64 = 54. * f64::powf(geo_const::EARTH_SEMI_MINOR_AXIS, 2.) * z_ecef_squared;
    let g_term: f64 = range_squared + (1. - geo_const::ECEF2LLA_E2) * z_ecef_squared
        - geo_const::ECEF2LLA_E2
            * (f64::powf(geo_const::EARTH_SEMI_MAJOR_AXIS, 2.)
                - f64::powf(geo_const::EARTH_SEMI_MINOR_AXIS, 2.));

    let c_term: f64 = geo_const::ECEF2LLA_EP22 * f_term * range_squared / f64::powf(g_term, 3.);
    let c_term_sqrt_mod: f64 = f64::sqrt(f64::powf(c_term, 2.) + 2. * c_term);
    let s_term: f64 = f64::powf(1. + c_term + c_term_sqrt_mod, 1. / 3.);
    let p_term: f64 =
        f_term / (3. * f64::powf(s_term + 1. / s_term + 1., 2.) * f64::powf(g_term, 2.));
    let q_term: f64 = f64::sqrt(1. + 2. * geo_const::ECEF2LLA_EP22 * p_term);
    let sqrt_mod2: f64 = f64::sqrt(
        0.5 * f64::powf(geo_const::EARTH_SEMI_MAJOR_AXIS, 2.) * (1. + 1. / q_term)
            - p_term * (1. - geo_const::ECEF2LLA_E2) * z_ecef_squared / (q_term * (1. + q_term))
            - 0.5 * p_term * range_squared,
    );
    let r0_term: f64 = -(p_term * geo_const::ECEF2LLA_E2 * range_) / (1. + q_term) + sqrt_mod2;
    let uv_subterm: f64 = f64::powf(range_ - geo_const::ECEF2LLA_E2 * r0_term, 2.);
    let u_term: f64 = f64::sqrt(uv_subterm + z_ecef_squared);
    let v_term: f64 = f64::sqrt(uv_subterm + (1. - geo_const::ECEF2LLA_E2) * z_ecef_squared);
    let z0_term: f64 = f64::powf(geo_const::EARTH_SEMI_MINOR_AXIS, 2.) * ecef.z
        / (geo_const::EARTH_SEMI_MAJOR_AXIS * v_term);
    let lat_rad: f64 = if range_ != 0. {
        f64::atan2(ecef.z + geo_const::ECEF2LLA_EP2 * z0_term, range_)
    } else {
        0.0
    };
    let lon_rad: f64 = f64::atan2(ecef.y, ecef.x);
    let alt: f64 = u_term
        * (1.
            - f64::powf(geo_const::EARTH_SEMI_MINOR_AXIS, 2.)
                / (geo_const::EARTH_SEMI_MAJOR_AXIS * v_term));

    let lat: f64 = f64::to_degrees(lat_rad);
    let lon: f64 = f64::to_degrees(lon_rad);
    return glam::DVec3::new(lat, lon, alt);
}

/// Converts ECEF to LLA using formulation from `pymap3d`:
/// https://github.com/geospace-code/pymap3d/blob/b4dcee4cd7a43641666cef840c97dd73d4e5ed61/src/pymap3d/ecef.py#L87
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
///
/// # Returns
///
/// * `lla` - Vector represented in LLA coordinates [[degrees-degrees-meters]].
pub fn ecef2lla_map3d(ecef: &glam::DVec3) -> glam::DVec3 {
    let r = f64::sqrt(ecef.x * ecef.x + ecef.y * ecef.y + ecef.z * ecef.z);
    let r2 = r * r;
    let u = f64::sqrt(
        0.5 * (r2 - geo_const::EARTH_E_2)
            + 0.5 * f64::hypot(r2 - geo_const::EARTH_E_2, 2.0 * geo_const::EARTH_E * ecef.z),
    );
    let hxy = f64::hypot(ecef.x, ecef.y);
    let hue = f64::hypot(u, geo_const::EARTH_E);

    // rust signum returns 1 for 0.0, but we need 0.0 here for sign
    let sign = if ecef.z != 0.0 {
        f64::signum(ecef.z)
    } else {
        0.0
    };
    let mut beta = std::f64::consts::FRAC_PI_2 * sign;

    if !almost::zero(u) && !almost::zero(hxy) {
        beta = f64::atan(hue / u * ecef.z / hxy);
        beta += ((geo_const::EARTH_SEMI_MINOR_AXIS * u - geo_const::EARTH_SEMI_MAJOR_AXIS * hue
            + geo_const::EARTH_E_2)
            * f64::sin(beta))
            / (geo_const::EARTH_SEMI_MAJOR_AXIS * hue * 1. / f64::cos(beta)
                - geo_const::EARTH_E_2 * f64::cos(beta))
    }

    let lat = f64::atan(
        geo_const::EARTH_SEMI_MAJOR_AXIS / geo_const::EARTH_SEMI_MINOR_AXIS * f64::tan(beta),
    );
    let lon = f64::atan2(ecef.y, ecef.x);

    let mut alt = f64::hypot(
        ecef.z - geo_const::EARTH_SEMI_MINOR_AXIS * f64::sin(beta),
        hxy - geo_const::EARTH_SEMI_MAJOR_AXIS * f64::cos(beta),
    );
    let inside = ecef.x * ecef.x / geo_const::EARTH_SEMI_MAJOR_AXIS_2
        + ecef.y * ecef.y / geo_const::EARTH_SEMI_MAJOR_AXIS_2
        + ecef.z * ecef.z / geo_const::EARTH_SEMI_MINOR_AXIS_2
        < 1.;
    alt *= if inside { -1. } else { 1.0 };
    return glam::DVec3::new(f64::to_degrees(lat), f64::to_degrees(lon), alt);
}

/// Converts ECEF to LLA.
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
///
/// # Returns
///
/// * `lla` - Vector represented in LLA coordinates [[degrees-degrees-meters]].
pub fn ecef2lla(ecef: &glam::DVec3) -> glam::DVec3 {
    return ecef2lla_ferarri(ecef);
}

/// Converts LLA to ECEF.
///
/// # Arguments
///
/// * `lla` - Vector represented in LLA coordinates [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
pub fn lla2ecef(lla: &glam::DVec3) -> glam::DVec3 {
    let lat = f64::to_radians(lla.x);
    let lon = f64::to_radians(lla.y);
    let alt = lla.z;

    let alt_correction = geo_const::EARTH_SEMI_MAJOR_AXIS_2
        / f64::hypot(
            geo_const::EARTH_SEMI_MAJOR_AXIS * f64::cos(lat),
            geo_const::EARTH_SEMI_MINOR_AXIS * f64::sin(lat),
        );
    let x = (alt_correction + lla.z) * f64::cos(lat) * f64::cos(lon);
    let y = (alt_correction + lla.z) * f64::cos(lat) * f64::sin(lon);
    let z = (alt_correction
        * f64::powf(
            geo_const::EARTH_SEMI_MINOR_AXIS / geo_const::EARTH_SEMI_MAJOR_AXIS,
            2.0,
        )
        + alt)
        * f64::sin(lat);
    return glam::DVec3::new(x, y, z);
}

// todo: need to test these
pub fn juliandate(year: i64, month: i64, day: i64, hour: i64, min: i64, sec: i64) -> f64 {
    let jd = 367. * year as f64 - ((7. * (year as f64 + ((month as f64 + 9.) / 12.))) / 4.)
        + (275. * month as f64 / 9.)
        + day as f64
        + 1721013.5
        + hour as f64 / 24.
        + min as f64 / 1440.
        + sec as f64 / 86400.;
    return jd;
}

pub fn juliandate_from_utc_str(utc_datetime_str: String, fmt: Option<String>) -> f64 {
    let datetime_fmt = fmt.unwrap_or("%Y-%m-%dT%H:%M:%S".to_string());
    let utc = NaiveDateTime::parse_from_str(&utc_datetime_str, datetime_fmt.as_str()).unwrap();
    return juliandate(
        utc.year() as i64,
        utc.month() as i64,
        utc.day() as i64,
        utc.hour() as i64,
        utc.minute() as i64,
        utc.second() as i64,
    );
}

/// Computes the linear velocity a body would feel on earth in reference to the stars
pub fn linear_velocity_from_earth_rotation(earth_pos: &glam::DVec3) -> glam::DVec3 {
    let rot_axis = glam::DVec3::new(0., 0., geo_const::EARTH_ANGULAR_VEL_RADPS);
    return rot_axis.cross(*earth_pos);
}

/// Converts ecef to eci assuming ecef and eci were aligned time_since_eci_lock seconds ago
/// This uses a simple constant rotation assumption for earth
pub fn ecef2eci(ecef: &glam::DVec3, time_since_eci_lock: f64) -> glam::DVec3 {
    let earth_rotation_angle: f64 = time_since_eci_lock * geo_const::EARTH_ANGULAR_VEL_RADPS;

    let eci = glam::DQuat::from_rotation_z(earth_rotation_angle) * (*ecef);
    return eci;
}

pub fn eci2ecef(eci: &glam::DVec3, time_since_eci_lock: f64) -> glam::DVec3 {
    let earth_rotation_angle: f64 = time_since_eci_lock * -geo_const::EARTH_ANGULAR_VEL_RADPS;
    let ecef = glam::DQuat::from_rotation_z(earth_rotation_angle) * (*eci);
    return ecef;
}

pub fn ecef2eci_j2000(
    ecef: &glam::DVec3,
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
) -> glam::DVec3 {
    let jd = juliandate_from_utc_str(utc_time_str, utc_time_str_fmt);
    let tu = jd - 2451545.0;
    let earth_rot_angle = std::f64::consts::TAU * (0.7790572732640 + 1.00273781191135448 * tu);
    return glam::DQuat::from_rotation_z(earth_rot_angle) * (*ecef);
}

pub fn eci2ecef_j2000(
    eci: &glam::DVec3,
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
) -> glam::DVec3 {
    let jd = juliandate_from_utc_str(utc_time_str, utc_time_str_fmt);
    let tu = jd - 2451545.0;
    let earth_rot_angle = std::f64::consts::TAU * (0.7790572732640 + 1.00273781191135448 * tu);
    return glam::DMat3::from_rotation_z(-earth_rot_angle) * (*eci);
}

/// Calculates the quaternion that yields an ENU to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [[degrees]].
/// * `lon_deg` - Longitude reference [[degrees]].
///
/// # Returns
///
/// * `quat` - Normalized ENU to ECEF quaternion.
pub fn enu2ecef_quat(lat_deg: f64, lon_deg: f64) -> glam::DQuat {
    let lat_rad: f64 = f64::to_radians(lat_deg);
    let lon_rad: f64 = f64::to_radians(lon_deg);

    let yaw = glam::DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2 + lon_rad);
    let roll = glam::DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2 - lat_rad);
    return yaw * roll;
}

/// Calculates the quaternion that yields an ECEF to ENU transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [[degrees]].
/// * `lon_deg` - Longitude reference [[degrees]].
///
/// # Returns
///
/// * `quat` - Normalized ECEF to ENU quaternion.
pub fn ecef2enu_quat(lat_deg: f64, lon_deg: f64) -> glam::DQuat {
    let enu2ecef_rot = enu2ecef_quat(lat_deg, lon_deg);
    return enu2ecef_rot.conjugate();
}

/// Calculates the quaternion that yields an NED to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [[degrees]].
/// * `lon_deg` - Longitude reference [[degrees]].
///
/// # Returns
///
/// * `quat` - Normalized NED to ECEF quaternion.
pub fn ned2ecef_quat(lat_deg: f64, lon_deg: f64) -> glam::DQuat {
    let enu2ecef_q = enu2ecef_quat(lat_deg, lon_deg);
    let enu2ecef_mat = glam::DMat3::from_quat(enu2ecef_q);

    let east = enu2ecef_mat.x_axis;
    let north = enu2ecef_mat.y_axis;
    let up = enu2ecef_mat.z_axis;
    let down = -up;

    #[rustfmt::skip]
    let ned_mat = glam::DMat3::from_cols(north, east, down);
    return glam::DQuat::from_mat3(&ned_mat);
}

/// Calculates the quaternion that yields an ECEF to NED transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [[degrees]].
/// * `lon_deg` - Longitude reference [[degrees]].
///
/// # Returns
///
/// * `quat` - Normalized ECEF to NED quaternion.
pub fn ecef2ned_quat(lat_deg: f64, lon_deg: f64) -> glam::DQuat {
    let ned2ecef_rot = ned2ecef_quat(lat_deg, lon_deg);
    return ned2ecef_rot.conjugate();
}

/// Calculates the direction cosine matrix that yields an ENU to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [[degrees]].
/// * `lon_deg` - Longitude reference [[degrees]].
///
/// # Returns
///
/// * `dcm` - ENU to ECEF direction cosine matrix.
pub fn enu2ecef_dcm(lat_deg: f64, lon_deg: f64) -> glam::DMat3 {
    let q = enu2ecef_quat(lat_deg, lon_deg);
    return glam::DMat3::from_quat(q);
}

/// Calculates the direction cosine matrix that yields an ECEF to ENU transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [[degrees]].
/// * `lon_deg` - Longitude reference [[degrees]].
///
/// # Returns
///
/// * `dcm` - ECEF to ENU direction cosine matrix.
pub fn ecef2enu_dcm(lat_deg: f64, lon_deg: f64) -> glam::DMat3 {
    let q = ecef2enu_quat(lat_deg, lon_deg);
    return glam::DMat3::from_quat(q);
}

/// Calculates the direction cosine matrix that yields an NED to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [[degrees]].
/// * `lon_deg` - Longitude reference [[degrees]].
///
/// # Returns
///
/// * `dcm` - NED to ECEF direction cosine matrix.
pub fn ned2ecef_dcm(lat_deg: f64, lon_deg: f64) -> glam::DMat3 {
    let q = ned2ecef_quat(lat_deg, lon_deg);
    return glam::DMat3::from_quat(q);
}

/// Calculates the direction cosine matrix that yields an ECEF to NED transformation at this LLA.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [[degrees]].
/// * `lon_deg` - Longitude reference [[degrees]].
///
/// # Returns
///
/// * `dcm` - ECEF to NED direction cosine matrix.
pub fn ecef2ned_dcm(lat_deg: f64, lon_deg: f64) -> glam::DMat3 {
    let q = ecef2ned_quat(lat_deg, lon_deg);
    return glam::DMat3::from_quat(q);
}

/// Converts ECEF to ENU.
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
/// * `lla_ref` - Reference latitude-longitude-altitude [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `enu` - Vector represented in ENU coordinates [[meters]].
pub fn ecef2enu(ecef: &glam::DVec3, lla_ref: &glam::DVec3) -> glam::DVec3 {
    let rot = ecef2enu_quat(lla_ref.x, lla_ref.y);
    return rot * (*ecef);
}

/// Converts ENU to ECEF.
///
/// # Arguments
///
/// * `enu` - Vector represented in ENU coordinates [[meters]].
/// * * `lla_ref` - Reference latitude-longitude-altitude [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
pub fn enu2ecef(enu: &glam::DVec3, lla_ref: &glam::DVec3) -> glam::DVec3 {
    let rot = enu2ecef_quat(lla_ref.x, lla_ref.y);
    return rot * (*enu);
}

/// Converts ECEF to NED.
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
/// * `lla_ref` - Reference latitude-longitude-altitude [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `ned` - Vector represented in NED coordinates [[meters]].
pub fn ecef2ned(ecef: &glam::DVec3, lla_ref: &glam::DVec3) -> glam::DVec3 {
    let rot = ecef2ned_quat(lla_ref.x, lla_ref.y);
    return rot * (*ecef);
}

/// Converts NED to ECEF.
///
/// # Arguments
///
/// * `ned` - Vector represented in NED coordinates [[meters]].
/// * `lla_ref` - Reference latitude-longitude-altitude [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
pub fn ned2ecef(ned: &glam::DVec3, lla_ref: &glam::DVec3) -> glam::DVec3 {
    let rot = ned2ecef_quat(lla_ref.x, lla_ref.y);
    return rot * (*ned);
}

/// Converts ENU to AER.
///
/// # Arguments
///
/// * `enu` - Vector represented in ENU coordinates [[meters]].
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [[degrees-degrees-meters]].
pub fn enu2aer(enu: &glam::DVec3) -> glam::DVec3 {
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
pub fn aer2enu(aer: &glam::DVec3) -> glam::DVec3 {
    let r = aer.z;
    let az = f64::to_radians(aer.x);
    let el = f64::to_radians(aer.y);
    let east_north = r * f64::cos(el);
    let east = east_north * f64::sin(az);
    let north = east_north * f64::cos(az);
    let up = r * f64::sin(el);
    return glam::DVec3::new(east, north, up);
}

/// Converts ECEF to AER.
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
/// * `ref_lla` - Reference latitude-longitude-altitude [[radians-radians-meters]].
///
/// # Returns
///
/// * `aer` - Vector represented in AER coordinates [[degrees-degrees-meters]].
pub fn ecef2aer(ecef: &glam::DVec3, lla_ref: &glam::DVec3) -> glam::DVec3 {
    let enu = ecef2enu(ecef, lla_ref);
    return enu2aer(&enu);
}

/// Converts AER to ECEF.
///
/// # Arguments
///
/// * `aer` - Vector represented in AER coordinates [[degrees-degrees-meters]].
/// * `ref_lla` - Reference latitude-longitude-altitude [[radians-radians-meters]].
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
pub fn aer2ecef(aer: &glam::DVec3, ref_lla: &glam::DVec3) -> glam::DVec3 {
    let enu = aer2enu(aer);
    return enu2ecef(&enu, &ref_lla) + lla2ecef(ref_lla);
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
pub fn ned2aer(ned: &glam::DVec3) -> glam::DVec3 {
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
pub fn aer2ned(aer: &glam::DVec3) -> glam::DVec3 {
    let enu = aer2enu(aer);
    return glam::DVec3::new(enu.y, enu.x, -enu.z);
}

/// Calculates heading angle from ENU.
///
/// # Arguments
///
/// * `enu` - Vector represented in ENU coordinates [[meters]].
///
/// # Returns
///
/// * `heading_deg` - Heading angle relative to true north [[degrees]].
pub fn enu2heading(enu: &glam::DVec3) -> f64 {
    return f64::atan2(enu.x, enu.y).to_degrees();
}

/// Computes the clockwise angle off north that a local body frame's X axis is pointing at a location on earth given a local2ecef dcm
/// 
/// # Arguments
/// 
/// * `ecef_dcm` - Local body frame expressed as a dcm
/// 
/// # Returns
/// 
/// *  `heading` - Clockwise angle off true north [[degrees]] of the body's x axis
pub fn ecef_dcm2heading(ecef_dcm: &glam::DMat3, lla_ref: &glam::DVec3) -> f64 {
    let ecef2enu = ecef2enu_dcm(lla_ref.x, lla_ref.y);
    let enu_dcm = ecef2enu * (*ecef_dcm);
    let forward = enu_dcm.x_axis;
    return enu2heading(&forward);
}

/// Computes the clockwise angle off north that a local body frame's X axis is pointing at a location on earth given a local2ecef quaternion
/// 
/// # Arguments
/// 
/// * `ecef_quat` - Local body frame expressed as a quaternion
/// 
/// # Returns
/// 
/// *  `heading` - Clockwise angle off true north [[degrees]] of the body's x axis
pub fn ecef_quat2heading(ecef_quat: &glam::DQuat, lla_ref: &glam::DVec3) -> f64 {
    let ecef_dcm = glam::DMat3::from_quat(*ecef_quat);
    return ecef_dcm2heading(&ecef_dcm, lla_ref);
}

/// Generates a uniform random point on the surface of a sphere.
///
/// # Arguments
///
/// * `radius` - Radius of sphere.
///
/// # Returns
///
/// * `vector` - Random point in R3.
pub fn rand_point_on_sphere(radius: f64) -> glam::DVec3 {
    let mut rng = rand::rng();
    let normal = Normal::new(0.0, 1.0).unwrap();

    return radius
        * (glam::DVec3::new(
            normal.sample(&mut rng),
            normal.sample(&mut rng),
            normal.sample(&mut rng),
        )
        .normalize());
}

/// Generates a uniform random ECEF point on the surface of a spherical Earth.
///
/// # Returns
///
/// * `ecef` - Random ECEF location [[meters]].
pub fn rand_ecef() -> glam::DVec3 {
    return rand_point_on_sphere(geo_const::EARTH_SEMI_MAJOR_AXIS);
}

/// Generates a uniform random LLA point. Altitude is generated in the domain [[0.0, 10000.0]].
///
/// # Returns
///
/// * `lla` - Random LLA location [[degrees-degrees-meters]].
pub fn rand_lla() -> glam::DVec3 {
    return glam::DVec3::new(
        util::lerp(-90.0, 90.0, rand::random()),
        util::lerp(-180.0, 180.0, rand::random()),
        util::lerp(0.0, 10000.0, rand::random()),
    );
}

#[derive(Debug, Clone)]
pub struct IllFormedDMSError {
    pub bad_dms: String,
}

impl fmt::Display for IllFormedDMSError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Got ill-formed degrees minutes seconds: {}. Should be format DD:MM:SS.SSSC where C is a cardinal direction [N, S, E, W]", 
            self.bad_dms
        )
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
/// use map3d::dms2dd;
/// use approx::relative_eq;
///
/// let dd = dms2dd("25:22:44.738N").unwrap();
/// assert!(relative_eq!(dd, 25.37909389, max_relative = 1e-8, epsilon = 1e-9));
/// ```
pub fn dms2dd(dms: &str) -> Result<f64, IllFormedDMSError> {
    let parts = dms.split(":").collect::<Vec<&str>>();

    if parts.len() == 3 {
        let sdeg = parts[0];
        let smin = parts[1];
        let sec_card = parts[2];
        if let Some(cardinal) = sec_card.chars().last() {
            let lower_card = &cardinal.to_lowercase().to_string()[..];
            let ssec = &sec_card[..sec_card.len() - 1];
            if let (Ok(deg), Ok(min), Ok(sec)) = (
                sdeg.parse::<f64>(),
                smin.parse::<f64>(),
                ssec.parse::<f64>(),
            ) {
                if vec!["n", "s", "e", "w"].contains(&lower_card) {
                    let mut dd = deg + min / 60. + sec / 3600.;
                    if lower_card == "s" || lower_card == "w" {
                        dd *= -1.;
                    }
                    return Ok(dd);
                }
            }
        }
    }
    return Err(IllFormedDMSError {
        bad_dms: dms.to_string(),
    });
}

/// Converts decimal degrees (DD) to degrees-minutes-seconds (DMS).
///
/// # Arguments
///
/// * `dd` - Decimal degrees [degrees].
/// * `is_lat` - Flat to denote if this decimal value describes a latitude [bool]
///
/// # Returns
///
/// * `dms` - Format "{Degrees}:{Minutes}:{Seconds}.{DecSeconds}{Direction}" where Direction is one of N, E, S, or W.
///
/// # Examples
/// ```
/// use map3d::dd2dms;
/// use approx::relative_eq;
///
/// let dms = dd2dms(25.37909389, true);
/// assert_eq!(dms, "25:22:44.738N");
/// ```
pub fn dd2dms(dd: f64, is_lat: bool) -> String {
    let dir: &str;

    if is_lat {
        if dd > 0. {
            dir = "N";
        } else {
            dir = "S";
        }
    } else {
        if dd > 0. {
            dir = "E";
        } else {
            dir = "W";
        }
    }

    let deg = f64::abs(dd) * 3600.;
    let (mnt, sec) = (deg / 60., deg % 60.);
    let (deg, min) = (mnt / 60., mnt % 60.);
    return format!("{:.0}:{:.0}:{:.3}{}", deg.floor(), min.floor(), sec, dir);
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
pub fn ll2dms(lat: f64, lon: f64) -> (String, String) {
    return (dd2dms(lat, true), dd2dms(lon, false));
}

#[derive(Debug, Clone)]
pub struct DomainError {
    pub var: String,
    pub value: f64,
    pub min: f64,
    pub max: f64,
    pub min_inclusive: bool, // If true, min is included in domain as an acceptable value.
    pub max_inclusive: bool, // If true, max is included in domain as an acceptable value.
}

impl fmt::Display for DomainError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let min_inequality: &str = if self.min_inclusive { "[" } else { "(" };
        let max_inequality: &str = if self.max_inclusive { "]" } else { ")" };

        write!(
            f,
            "{} must be in domain {}{}, {}{} but received {}",
            self.var, min_inequality, self.min, self.max, max_inequality, self.value
        )
    }
}

/// Calculates the LLA location that is a fixed range and bearing from a reference LLA. This function uses an iterative
/// solution to determine outputs using the WGS84 ellipsoidal Earth model.
///
/// See reference:
/// https://en.wikipedia.org/wiki/Vincenty%27s_formulae.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [[degrees]].
/// * `lon_deg` - Longitude reference [[degrees]].
/// * `range_m` - Range (i.e., distance) from point A to point B [[meters]].
/// * `bearing_deg` - Bearing (i.e., azimuth) from point A to point B relative to true north [[degrees]].
/// * `abs_tol` - Absolute tolerance used for convergence.
/// * `max_iters` - Maximum possible number of iterations before early termination.
///
/// # Returns
///
/// A tuple `(lat_deg, lon_deg)` where:
/// * `lat_deg` - Latitude location [[degrees]].
/// * `lon_deg` - Longitude location [[degrees]].
pub fn vincenty_direct(
    lat_deg: f64,
    lon_deg: f64,
    range_m: f64,
    bearing_deg: f64,
    atol: f64,
    max_iters: u16,
) -> Result<(f64, f64), DomainError> {
    if lat_deg.abs() > 90.0 {
        return Err(DomainError {
            var: "lat_deg".into(),
            value: lat_deg,
            min: -90.0,
            max: 90.0,
            min_inclusive: true,
            max_inclusive: true,
        });
    }

    if range_m < 0.0 {
        return Err(DomainError {
            var: "range_m".into(),
            value: range_m,
            min: 0.0,
            max: INFINITY,
            min_inclusive: true,
            max_inclusive: false,
        });
    }

    if atol <= 0.0 {
        return Err(DomainError {
            var: "atol".into(),
            value: atol,
            min: 0.0,
            max: INFINITY,
            min_inclusive: false,
            max_inclusive: false,
        });
    }

    let lat_rad = lat_deg.to_radians();
    let lon_rad = lon_deg.to_radians();
    let bearing_rad = bearing_deg.to_radians();

    let u1 = ((1.0 - geo_const::EARTH_FLATTENING_FACTOR) * lat_rad.tan()).atan();
    let s1 = u1.tan().atan2(bearing_rad.cos());
    let sina = u1.cos() * bearing_rad.sin();
    let cos2a = 1.0 - sina.powi(2);
    let u2 = cos2a * (geo_const::EARTH_SEMI_MAJOR_AXIS_2 - geo_const::EARTH_SEMI_MINOR_AXIS_2)
        / geo_const::EARTH_SEMI_MINOR_AXIS_2;

    let k1 = ((1.0 + u2).sqrt() - 1.0) / ((1.0 + u2).sqrt() + 1.0);
    let a = (1.0 + 0.25 * k1.powi(2)) / (1.0 - k1);
    let b = k1 * (1.0 - 3. / 8.0 * k1.powi(2));

    // Loop variables that get updated throughout iteration.
    let mut steps: u16 = 0;
    let mut sigma = range_m / (geo_const::EARTH_SEMI_MINOR_AXIS * a);
    let mut twosigm = 2.0 * s1 + sigma;
    let mut cos2sm = twosigm.cos();
    let mut sins = sigma.sin();
    let mut coss = sigma.cos();

    loop {
        let delta_sigma_term_1 = coss * (-1.0 + 2.0 * cos2sm.powi(2));
        let delta_sigma_term_2 = (-3.0 + 4.0 * sins.powi(2)) * (-3.0 + 4.0 * cos2sm.powi(2));
        let delta_sigma_term_3 = b / 6.0 * cos2sm * delta_sigma_term_2;
        let delta_sigma_term_4 = delta_sigma_term_1 - delta_sigma_term_3;
        let delta_sigma = b * sins * (cos2sm + 0.25 * b * delta_sigma_term_4);
        let sigma_old = sigma;

        sigma = range_m / (geo_const::EARTH_SEMI_MINOR_AXIS * a) + delta_sigma;
        twosigm = 2.0 * s1 + sigma;
        cos2sm = twosigm.cos();
        sins = sigma.sin();
        coss = sigma.cos();

        steps += 1;
        if ((sigma_old - sigma).abs() < atol) || (steps >= max_iters) {
            break;
        }
    }

    let phi2_term_1 = u1.sin() * coss + u1.cos() * sins * bearing_rad.cos();
    let phi2_term_2 = (u1.sin() * sins - u1.cos() * coss * bearing_rad.cos()).powi(2);
    let phi2_term_3 =
        (1.0 - geo_const::EARTH_FLATTENING_FACTOR) * (sina.powi(2) + phi2_term_2).sqrt();
    let phi2 = phi2_term_1.atan2(phi2_term_3);

    let lam =
        (sins * bearing_rad.sin()).atan2(u1.cos() * coss - u1.sin() * sins * bearing_rad.cos());
    let c = geo_const::EARTH_FLATTENING_FACTOR / 16.0
        * cos2a
        * (4.0 + geo_const::EARTH_FLATTENING_FACTOR * (4.0 - 3.0 * cos2a));

    let l_term_1 = sigma + c * sins * (cos2sm + c * coss * (-1.0 + 2.0 * cos2sm.powi(2)));
    let l = lam - (1.0 - c) * geo_const::EARTH_FLATTENING_FACTOR * sina * l_term_1;
    let l2 = l + lon_rad;

    Ok((phi2.to_degrees(), l2.to_degrees()))
}

/// Calculates range and bearings between two latitude-longitude points. This function uses an iterative solution to
/// determine outputs using the WGS84 ellipsoidal Earth model.
///
/// See reference:
/// https://en.wikipedia.org/wiki/Vincenty%27s_formulae.
///
/// # Arguments
///
/// * `lat_a_deg` - Latitude point A [[degrees]].
/// * `lon_a_deg` - Longitude point A [[degrees]].
/// * `lat_b_deg` - Latitude point A [[degrees]].
/// * `lon_b_deg` - Longitude point A [[degrees]].
/// * `atol` - Absolute tolerance used for convergence.
/// * `max_iters` - Maximum possible number of iterations before early termination.
///
/// # Returns
///
/// A tuple `(range_m, bearing_ab_deg, bearing_ba_deg)` where:
/// * `range_m` - Range (i.e., distance) from point A to point B [[meters]].
/// * `bearing_ab_deg` - Bearing (i.e., azimuth) from point A to point B relative to true north [[degrees]].
/// * `bearing_ba_deg` - Bearing (i.e., azimuth) from point B to point A relative to true north [[degrees]].
pub fn vincenty_inverse(
    lat_a_deg: f64,
    lon_a_deg: f64,
    lat_b_deg: f64,
    lon_b_deg: f64,
    atol: f64,
    max_iters: u16,
) -> Result<(f64, f64, f64), DomainError> {
    if lat_a_deg.abs() > 90.0 {
        return Err(DomainError {
            var: "lat_a_deg".into(),
            value: lat_a_deg,
            min: -90.0,
            max: 90.0,
            min_inclusive: true,
            max_inclusive: true,
        });
    }

    if lat_b_deg.abs() > 90.0 {
        return Err(DomainError {
            var: "lat_b_deg".into(),
            value: lat_b_deg,
            min: -90.0,
            max: 90.0,
            min_inclusive: true,
            max_inclusive: true,
        });
    }

    if atol <= 0.0 {
        return Err(DomainError {
            var: "atol".into(),
            value: atol,
            min: 0.0,
            max: INFINITY,
            min_inclusive: false,
            max_inclusive: false,
        });
    }

    let lat_a_rad = lat_a_deg.to_radians();
    let lon_a_rad = lon_a_deg.to_radians();
    let lat_b_rad = lat_b_deg.to_radians();
    let lon_b_rad = lon_b_deg.to_radians();

    let u1 = ((1.0 - geo_const::EARTH_FLATTENING_FACTOR) * lat_a_rad.tan()).atan();
    let u1_sin = u1.sin();
    let u1_cos = u1.cos();

    let u2 = ((1.0 - geo_const::EARTH_FLATTENING_FACTOR) * lat_b_rad.tan()).atan();
    let u2_sin = u2.sin();
    let u2_cos = u2.cos();

    let l = lon_b_rad - lon_a_rad;

    // Early termination if the provided LLA points are coincident.
    if ((lat_a_rad - lat_b_rad).abs() < vincenty_const::INVERSE_SIN_SIGMA_TOL)
        && (l.abs() < vincenty_const::INVERSE_SIN_SIGMA_TOL)
    {
        return Ok((0.0, 0.0, 0.0));
    }

    // Loop variables that get updated throughout iteration.
    let mut steps: u16 = 0;
    let mut lam = l;
    let mut cos2sm;
    let mut cos2a;
    let mut sigma;
    let mut sin_sigma;
    let mut cos_sigma;

    loop {
        let sin_sigma_term_1 = (u2_cos * lam.sin()).powi(2);
        let sin_sigma_term_2 = (u1_cos * u2_sin - u1_sin * u2_cos * lam.cos()).powi(2);
        sin_sigma = (sin_sigma_term_1 + sin_sigma_term_2).sqrt();

        if sin_sigma < vincenty_const::INVERSE_SIN_SIGMA_TOL {
            if (lat_a_rad - lat_b_rad).abs() < vincenty_const::INVERSE_SIN_SIGMA_TOL {
                // Handles when points that share common latitudes converge to be coincident during iteration.
                return Ok((0.0, 0.0, 0.0));
            } else {
                // Else the points are anti-podal and a small perturbation is added to avoid divide by zero.
                sin_sigma = vincenty_const::INVERSE_SIN_SIGMA_TOL;
            }
        }

        cos_sigma = u1_sin * u2_sin + u1_cos * u2_cos * lam.cos();
        sigma = sin_sigma.atan2(cos_sigma);
        let sina = u1_cos * u2_cos * lam.sin() / sigma.sin();

        // Handle case where both points are on the equator, yielding a divide by zero error.
        cos2a = 1.0 - sina.powi(2);
        if cos2a.abs() < vincenty_const::INVERSE_COS2A_TOL {
            cos2sm = 0.0;
        } else {
            cos2sm = cos_sigma - 2.0 * u1_sin * u2_sin / cos2a;
        }

        let c = geo_const::EARTH_FLATTENING_FACTOR / 16.0
            * cos2a
            * (4.0 + geo_const::EARTH_FLATTENING_FACTOR * (4.0 - 3.0 * cos2a));
        let lam_old = lam;
        let lam_end_term = cos2sm + c * cos_sigma * (-1.0 + 2.0 * cos2sm.powi(2));
        lam = l
            + (1.0 - c)
                * geo_const::EARTH_FLATTENING_FACTOR
                * sina
                * (sigma + c * sin_sigma * lam_end_term);
        steps += 1;
        if ((lam_old - lam).abs() < atol) || (steps > max_iters) {
            break;
        }
    }

    let g2 = cos2a * (geo_const::EARTH_SEMI_MAJOR_AXIS_2 - geo_const::EARTH_SEMI_MINOR_AXIS_2)
        / geo_const::EARTH_SEMI_MINOR_AXIS_2;
    let k1 = ((1.0 + g2).sqrt() - 1.0) / ((1.0 + g2).sqrt() + 1.0);
    let a = (1.0 + 0.25 * k1.powi(2)) / (1.0 - k1);
    let b = k1 * (1.0 - 3.0 / 8.0 * k1.powi(2));

    let delta_sigma_term_1 = cos_sigma * (-1.0 + 2.0 * cos2sm.powi(2));
    let delta_sigma_term_2 = (-3.0 + 4.0 * sin_sigma.powi(2)) * (-3.0 + 4.0 * cos2sm.powi(2));
    let delta_sigma_term_3 = delta_sigma_term_1 - b / 6.0 * cos2sm * delta_sigma_term_2;
    let delta_sigma = b * sin_sigma * (cos2sm + 0.25 * b * delta_sigma_term_3);

    // For clarity, pi is added to the bearing from B to A so that the angle return is the azimuth from B to A, rather
    // than the forward azimuth from A's perspective. However, to keep both angles in the same domain (+/- pi), the
    // modified angle is wrapped.
    let range_m = geo_const::EARTH_SEMI_MINOR_AXIS * a * (sigma - delta_sigma);
    let bearing_ab_rad = (u2_cos * lam.sin()).atan2(u1_cos * u2_sin - u1_sin * u2_cos * lam.cos());
    let bearing_ba_rad = util::wrap_to_pi(
        (u1_cos * lam.sin()).atan2(-u1_sin * u2_cos + u1_cos * u2_sin * lam.cos()) + PI,
    );

    Ok((
        range_m,
        bearing_ab_rad.to_degrees(),
        bearing_ba_rad.to_degrees(),
    ))
}

#[cfg(test)]
mod geotests {
    use super::*;
    use approx::relative_eq;
    use rstest::*;

    fn assert_vecs_close(a: &glam::DVec3, b: &glam::DVec3, tol: f64) {
        assert!(almost::equal_with(a.x, b.x, tol));
        assert!(almost::equal_with(a.y, b.y, tol));
        assert!(almost::equal_with(a.z, b.z, tol));
    }

    #[fixture]
    fn ecef_fixture() -> (Vec<glam::DVec3>, Vec<glam::DVec3>) {
        let ecefs: Vec<glam::DVec3> = vec![
            glam::DVec3::new(-4395937.137069274, -5292832.080918098, 269071.20488286763),
            glam::DVec3::new(-17425.064377540722, -2006865.517897409, 7930762.472554953),
            glam::DVec3::new(-3895837.31483683, 6644097.950743062, -1824545.3930313005),
            glam::DVec3::new(-6409133.120866153, 4558772.702384297, -1947061.2076449227),
            glam::DVec3::new(851485.3223891761, -2568595.6166770314, -8943557.577897426),
            glam::DVec3::new(5691809.8303289935, -7030732.321824082, -8330125.529865682),
            glam::DVec3::new(2516433.37963281, -1241849.192556627, -6760285.815142313),
            glam::DVec3::new(-3482763.8631693274, 6769587.309460899, -944640.6623275336),
            glam::DVec3::new(-370523.61786744185, 6024008.984975778, 8509531.670449838),
            glam::DVec3::new(5313476.383658681, -9825896.959403213, -1741391.5667025768),
            glam::DVec3::new(-8908541.706026632, -7037043.26505645, 6552883.707257189),
            glam::DVec3::new(-617141.4009794481, 2060754.5193012804, 8394970.341717768),
            glam::DVec3::new(8538245.25323937, 4747903.057412151, -9958019.246258922),
            glam::DVec3::new(-2108304.0336288405, -1654639.056214511, -9162254.95404531),
            glam::DVec3::new(-9223445.909886952, -400765.09915581346, -9900123.17700839),
            glam::DVec3::new(-7688902.383860592, -4471753.673588827, -5826850.3124411125),
            glam::DVec3::new(-3018693.051038252, -5388736.217988148, -5258460.323543303),
            glam::DVec3::new(-9288375.018009178, -2127487.6902046744, 6460020.61877924),
            glam::DVec3::new(-144971.8057209216, 114606.55915352888, 7971798.167930942),
            glam::DVec3::new(6828101.960543122, 693885.4050173741, -1196749.1985590179),
            glam::DVec3::new(-8578157.642460234, -1842531.7364011942, 6845094.434824787),
            glam::DVec3::new(5787354.780059811, 1210116.2836728748, -4726770.355619501),
            glam::DVec3::new(4193285.538259528, -1970166.3550874572, 7412301.116472796),
            glam::DVec3::new(3890132.340585826, 1161874.8197520673, -7909529.461251948),
            glam::DVec3::new(-7403258.820000752, -2903792.6257274374, -8826380.396521661),
            glam::DVec3::new(1857337.7916966896, 368102.4709657077, -6155560.052247517),
            glam::DVec3::new(-7588845.593718505, -8209442.364279032, 8265779.358689017),
            glam::DVec3::new(-5108159.12198603, -8476495.093040336, 6352145.996764505),
            glam::DVec3::new(-186001.42373996228, -7630898.02119923, -9721624.592627238),
            glam::DVec3::new(2369638.1324996054, 4073593.1322049536, 452913.52990143374),
            glam::DVec3::new(449824.9360387679, 4166398.4307631515, 2736272.9522982505),
            glam::DVec3::new(9965957.90734066, 2221724.6208532117, -6853904.619168903),
            glam::DVec3::new(-4478176.754602823, 1036869.5552951898, -9423720.838865533),
            glam::DVec3::new(2097024.4875351437, -9881162.142519716, 4949580.930534031),
            glam::DVec3::new(201204.94145997614, 9498512.914456967, 3515229.3174460693),
            glam::DVec3::new(-2060584.097762437, 3861553.560423618, -7614373.714063268),
            glam::DVec3::new(1757425.2753162049, 3398544.52313062, 8492215.282477114),
            glam::DVec3::new(-9047342.859610673, -8441792.192733321, -4469448.824678039),
            glam::DVec3::new(-487921.69639001414, -6030301.279856028, -9260258.22696507),
            glam::DVec3::new(-2744053.779757822, 2376587.0533640455, -3680945.978733376),
            glam::DVec3::new(-1815727.9597172216, 1953269.055585172, 6538261.825529313),
            glam::DVec3::new(8720682.736849941, -3489723.916849705, -5774441.696210405),
            glam::DVec3::new(-2933009.790774938, -5440069.166919639, -8623652.84907917),
            glam::DVec3::new(549449.791821098, -2920680.440199934, -3534971.5976333166),
            glam::DVec3::new(-5140412.492418254, -7429509.788924876, -1444380.0341157485),
            glam::DVec3::new(9626007.606757686, 8719399.041604526, 1697464.7512210696),
            glam::DVec3::new(-9127014.835284937, -6298505.842698975, -773826.4704206288),
            glam::DVec3::new(1842921.888856599, 8121796.140805811, 6227066.092612989),
            glam::DVec3::new(5138399.743888708, 9322686.007429656, -2555719.5912182704),
            glam::DVec3::new(9598896.75147951, 5044493.199291278, -6248509.982967714),
        ];
        let llas: Vec<glam::DVec3> = vec![
            glam::DVec3::new(2.253516806471046, -129.71116955184112, 507441.10318602197),
            glam::DVec3::new(75.87000157402547, -90.49747108091655, 1822723.22233205),
            glam::DVec3::new(-13.396850991057667, 120.38568018022004, 1538213.4642196773),
            glam::DVec3::new(-13.975248661411722, 144.5760168892664, 1725598.1936111697),
            glam::DVec3::new(-73.23828849316851, -71.65972404561161, 2985437.311158756),
            glam::DVec3::new(-42.74052199178427, -51.00770324601096, 5928794.414992734),
            glam::DVec3::new(-67.57498967210243, -26.26614362057283, 959684.6950130499),
            glam::DVec3::new(-7.112487646164081, 117.22455673918091, 1293518.4109493706),
            glam::DVec3::new(54.76454334396985, 93.51970396395149, 4068652.2450541486),
            glam::DVec3::new(-8.893647137325992, -61.59714074842926, 4927844.7237863885),
            glam::DVec3::new(30.07510912049895, -141.69405513524464, 6735317.147520137),
            glam::DVec3::new(75.69522425715692, 106.67156561450872, 2308138.501369687),
            glam::DVec3::new(-45.63516031581972, 29.077333155761565, 7582905.476120061),
            glam::DVec3::new(-73.76417124368244, -141.8744698912028, 3187753.7972916476),
            glam::DVec3::new(-47.08986703197784, -177.51202373059226, 7170110.832990964),
            glam::DVec3::new(-33.334219053050205, -149.81834070560083, 4261638.047698805),
            glam::DVec3::new(-40.55844825566602, -119.256926093939, 1742735.252764878),
            glam::DVec3::new(34.23382312320651, -167.09902807775964, 5140859.543690856),
            glam::DVec3::new(88.67910855596926, 141.67208580697599, 1617176.124676821),
            glam::DVec3::new(-9.951000598269978, 5.80259213124441, 589322.5756239324),
            glam::DVec3::new(38.067270677905, -167.87743403521839, 4758084.056677728),
            glam::DVec3::new(-38.79866773869741, 11.810196086116186, 1199912.009335559),
            glam::DVec3::new(58.11840077026344, -25.165937921189123, 2378386.964989038),
            glam::DVec3::new(-62.94048624737819, 16.62942891748623, 2529453.921751855),
            glam::DVec3::new(-48.084398711074584, -158.58331540347453, 5514132.784860136),
            glam::DVec3::new(-73.00848065406132, 11.210083031167493, 81603.365385446),
            glam::DVec3::new(36.56190250218874, -132.75043328084922, 7532975.424592685),
            glam::DVec3::new(32.78902393868271, -121.07427361795573, 5387958.701666567),
            glam::DVec3::new(-51.95818468723161, -91.39629526176816, 5995326.962900662),
            glam::DVec3::new(5.539216363414069, 59.81313129551833, -1643546.9126202406),
            glam::DVec3::new(33.367417431775394, 83.83793347365611, -1366875.6175801605),
            glam::DVec3::new(-33.963908882404155, 12.56752085132872, 5926171.518468513),
            glam::DVec3::new(-64.09003114845517, 166.96355233242235, 4124167.2847124636),
            glam::DVec3::new(26.190941042810294, -78.01820568227308, 4874709.392305468),
            glam::DVec3::new(20.38330525588493, 88.78649739621953, 3754550.8161831154),
            glam::DVec3::new(-60.228949370259244, 118.08506369211479, 2420676.065792122),
            glam::DVec3::new(65.84506256608404, 62.65601109433141, 2953957.9933096142),
            glam::DVec3::new(-19.918996193014078, -136.9830393254037, 6780862.795777614),
            glam::DVec3::new(-56.943542406173144, -94.62581967163953, 4698278.924516642),
            glam::DVec3::new(-45.63501439769118, 139.10461559970705, -1197409.76205113),
            glam::DVec3::new(67.93118253875696, 132.9100464016069, 701440.670716858),
            glam::DVec3::new(-31.68079101252089, -21.809626299383666, 4653736.780262814),
            glam::DVec3::new(-54.48102155235577, -118.3312980633975, 4245634.217446316),
            glam::DVec3::new(-50.206653953803524, -79.34581589819817, -1747312.1358781399),
            glam::DVec3::new(-9.125140350907303, -124.67901908506032, 2771588.2018704447),
            glam::DVec3::new(7.470140696990517, 42.17081118035693, 6720670.269056683),
            glam::DVec3::new(-4.007031763998265, -145.39064333427916, 4738278.426321056),
            glam::DVec3::new(36.898669089623674, 77.21546306700436, 4028403.093408107),
            glam::DVec3::new(-13.551349707628185, 61.137694236757405, 4570511.41330536),
            glam::DVec3::new(-30.036801286738232, 27.723202807325016, 6142375.721604907),
        ];
        return (ecefs, llas);
    }
    #[fixture]
    fn lla_fixture() -> (Vec<glam::DVec3>, Vec<glam::DVec3>) {
        let llas: Vec<glam::DVec3> = vec![
            glam::DVec3::new(-56.66172812377343, 155.68130312212668, 718875820.2943813),
            glam::DVec3::new(-33.26507252871781, -31.964572932142886, 7065343507.721147),
            glam::DVec3::new(6.371772880304988, -59.23405806377991, 8352922369.965223),
            glam::DVec3::new(-70.74335211522242, 120.07092031064208, 4526779918.14558),
            glam::DVec3::new(-24.461510525146508, -59.62407504106517, 2420545872.95108),
            glam::DVec3::new(-72.3152549895963, 101.64767964829839, 3795852142.154489),
            glam::DVec3::new(50.1705938605771, 105.40775429487171, 7390834021.2357435),
            glam::DVec3::new(-32.30257694254681, -135.9790191808388, 2106745558.3293736),
            glam::DVec3::new(-19.934988367833313, 65.33934397460283, 6927358798.01723),
            glam::DVec3::new(23.702048312802475, 150.7801192499238, 4151592272.73058),
            glam::DVec3::new(-44.139744893078216, -10.394871262469564, 1308050706.656194),
            glam::DVec3::new(80.33315640988386, -167.00766083959087, 4853517330.590896),
            glam::DVec3::new(89.5832033021365, -146.3731250375061, 5541861449.666565),
            glam::DVec3::new(6.002327772952086, 178.1723518113481, 8243112160.578481),
            glam::DVec3::new(-30.240836896769675, 5.48046771739655, 774183075.5164747),
            glam::DVec3::new(-66.50252282873791, -130.9736602675406, 6467951118.810617),
            glam::DVec3::new(32.244715738302816, 134.35132917429144, 8312562383.178297),
            glam::DVec3::new(-40.929722118941775, -166.0099481725839, 6216809556.871332),
            glam::DVec3::new(-3.161999721921518, -121.6362613721991, 1686482297.1376824),
            glam::DVec3::new(70.69015001986463, -18.77422907679727, 4347545178.616857),
            glam::DVec3::new(-77.89686967839751, 76.8590940393218, 275547474.5753206),
            glam::DVec3::new(56.70256197263234, 31.487362633295106, 3351651550.650465),
            glam::DVec3::new(-8.537166774081555, 124.9300397881874, 6964157346.925901),
            glam::DVec3::new(-3.3883037953841466, 159.9876304913834, 5824233554.097087),
            glam::DVec3::new(3.3173719339601604, 19.53837842565602, 7727952079.516271),
            glam::DVec3::new(-29.656043118692313, 50.423126817903324, 3109749957.585276),
            glam::DVec3::new(-3.317182434873658, 115.18183094259633, 6783718307.968782),
            glam::DVec3::new(87.99594613540816, 113.8812711654927, 6548397030.25793),
            glam::DVec3::new(42.155897640035505, -7.385067279532194, 4223681045.2455497),
            glam::DVec3::new(-45.66421911004391, 165.75779873561737, 9231088702.94254),
            glam::DVec3::new(74.98669309517001, -139.02652259218758, 7752650661.051985),
            glam::DVec3::new(0.04388642130096798, 139.12251298740722, 9856972131.89139),
            glam::DVec3::new(18.913840578810465, -152.22766889844678, 8674973608.740688),
            glam::DVec3::new(-70.71679219133352, 168.7938671723137, 2559962939.2985635),
            glam::DVec3::new(-17.840267641554703, 145.15346668909052, 9666183830.211079),
            glam::DVec3::new(7.81829152155791, 178.60319210639625, 2434544733.5319457),
            glam::DVec3::new(-86.60171868991569, -144.4301374069363, 8315174907.876453),
            glam::DVec3::new(-64.583522424254, 117.42569962019991, 2137778660.492624),
            glam::DVec3::new(-53.497097618994985, 66.73409062380492, 6968275498.757018),
            glam::DVec3::new(39.85170162247488, 82.86455367169816, 5083534728.543176),
            glam::DVec3::new(-89.14033256764529, 3.817737674123464, 8488834909.559485),
            glam::DVec3::new(-8.097151555351061, -60.04675219817315, 1794800252.0766892),
            glam::DVec3::new(60.35525557045381, 137.40735372191648, 2643745363.528964),
            glam::DVec3::new(-33.80752224309167, 151.33853754701425, 4511336080.320615),
            glam::DVec3::new(44.49769217327577, 35.65986371715448, 7747124179.421991),
            glam::DVec3::new(78.6881809933453, 153.6339574621485, 1262173554.861432),
            glam::DVec3::new(50.934753774135714, 48.68011259868561, 3940700238.395185),
            glam::DVec3::new(23.139059023558076, -30.9794825232527, 3732066418.3197985),
            glam::DVec3::new(-60.66725009582464, -27.905389704786387, 5654168566.610009),
            glam::DVec3::new(-89.52247206976813, 49.3889940119827, 2084170969.4723942),
        ];
        let ecefs: Vec<glam::DVec3> = vec![
            glam::DVec3::new(-363226337.8378485, 164145694.11941433, -605883225.5202017),
            glam::DVec3::new(5016417802.618961, -3130294546.8914347, -3878912812.734696),
            glam::DVec3::new(4249637013.1107903, -7138476691.558969, 927704883.3233159),
            glam::DVec3::new(-749123383.4568931, 1293818655.7673783, -4279508924.526832),
            glam::DVec3::new(1117071280.1290438, -1905836230.5036347, -1004929049.7808872),
            glam::DVec3::new(-233195557.85231522, 1131259987.718429, -3622523730.6242113),
            glam::DVec3::new(-1258810196.8083477, 4567666948.353717, 5680702139.465385),
            glam::DVec3::new(-1284356369.4257667, -1241197752.6029196, -1129213367.408031),
            glam::DVec3::new(2719703477.4990993, 5923770937.923891, -2364069442.8993587),
            glam::DVec3::new(-3322780364.3305306, 1858552546.759458, 1671407285.5992749),
            glam::DVec3::new(927816843.0914981, -170200408.00971267, -915359858.8829931),
            glam::DVec3::new(-795180643.7592813, -183469932.25196317, 4790867197.061249),
            glam::DVec3::new(-33606494.82719547, -22350830.551816523, 5548071401.239205),
            glam::DVec3::new(-8200090300.39463, 261659157.03025025, 862635420.7610741),
            glam::DVec3::new(671261500.4702597, 64404182.064525165, -393099795.1348588),
            glam::DVec3::new(-1692640892.2722268, -1948969436.132962, -5937439805.014645),
            glam::DVec3::new(-4918541001.068794, 5031191125.999227, 4438438970.340508),
            glam::DVec3::new(-4562247156.197555, -1136654630.154329, -4076992529.3376484),
            glam::DVec3::new(-886595545.8633595, -1439098934.8425846, -93374639.72836758),
            glam::DVec3::new(1363144037.4345145, -463368006.7734003, 4108967125.721511),
            glam::DVec3::new(13439868.383440318, 57568179.5971645, -275637077.97071016),
            glam::DVec3::new(1572069597.6960115, 962888601.5025085, 2806725105.8847656),
            glam::DVec3::new(-3946937207.6022162, 5651485039.359078, -1034776257.2819849),
            glam::DVec3::new(-5468975219.234577, 1991881397.6015773, -344601758.5886673),
            glam::DVec3::new(7276755652.2802105, 2582321037.9901214, 447557846.6049511),
            glam::DVec3::new(1725273328.2316494, 2087211973.2923207, -1541817056.8399067),
            glam::DVec3::new(-2884293340.9078116, 6134483975.098778, -392895716.5994358),
            glam::DVec3::new(-92799096.46533564, 209598007.20691597, 6550744575.743122),
            glam::DVec3::new(3109827878.675527, -403071979.06005263, 2838982707.422298),
            glam::DVec3::new(-6257302596.723574, 1588245806.4929829, -6607134865.92882),
            glam::DVec3::new(-1517523803.7477174, -1317930511.8018465, 7494157658.802485),
            glam::DVec3::new(-7457782245.210336, 6455006230.264203, 7554923.466331962),
            glam::DVec3::new(-7266578347.596589, -3826748677.7182827, 2814011879.0678635),
            glam::DVec3::new(-831350760.9757568, 164704360.3558542, -2422341250.456174),
            glam::DVec3::new(-7556421765.651863, 5260956590.476351, -2963316063.6143136),
            glam::DVec3::new(-2417514993.952572, 58948027.124627456, 332037521.0480978),
            glam::DVec3::new(-401231813.01364976, -286934665.0235125, -8306899057.677289),
            glam::DVec3::new(-423873876.7793667, 816838646.0451242, -1936605057.1374884),
            glam::DVec3::new(1638840880.6982732, 3811586986.433357, -5606389740.5359535),
            glam::DVec3::new(485379156.6587149, 3877298386.9417615, 3261608187.700436),
            glam::DVec3::new(127175105.33258279, 8486506.652578589, -8494235452.686916),
            glam::DVec3::new(890350698.4192543, -1545042792.5320225, -253693647.0788635),
            glam::DVec3::new(-965000269.7722342, 887134681.2380865, 2303222856.352874),
            glam::DVec3::new(-3293865707.0306754, 1800460317.3511906, -2513657365.399923),
            glam::DVec3::new(4493418846.659723, 3224078407.769552, 5434256241.694285),
            glam::DVec3::new(-222944108.97574744, 110505773.3995093, 1243887305.7373564),
            glam::DVec3::new(1642387205.8161027, 1868181939.908915, 3064602177.6773486),
            glam::DVec3::new(2947321316.0621667, -1769493145.9165955, 1469058917.9567044),
            glam::DVec3::new(2450561222.4285192, -1297800536.1928244, -4934781507.158352),
            glam::DVec3::new(11341319.896049185, 13227002.021572359, -2090455113.763534),
        ];
        return (llas, ecefs);
    }
    #[fixture]
    fn ecef2enu_fixture() -> (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>) {
        let ecef_ref_tuple: Vec<(glam::DVec3, glam::DVec3)> = vec![
            (
                glam::DVec3::new(91619.6484735217, 6925836.249227896, 5957639.1833251435),
                glam::DVec3::new(-50.173150092148774, -112.00193826311323, 0.0),
            ),
            (
                glam::DVec3::new(247424.65845609456, 6213457.926448222, -4561090.24503099),
                glam::DVec3::new(82.57782955532258, 179.8856930744933, 0.0),
            ),
            (
                glam::DVec3::new(4192418.204552155, 7351021.414750714, 1979901.0055067968),
                glam::DVec3::new(62.457227071538824, 55.92173622713477, 0.0),
            ),
            (
                glam::DVec3::new(-8588031.646098651, -7486734.193169072, -9230929.386199847),
                glam::DVec3::new(28.342454025165626, -92.0830991512501, 0.0),
            ),
            (
                glam::DVec3::new(1427216.04466255, 9403604.322131746, -963231.1375281066),
                glam::DVec3::new(30.516610115227095, -89.90471953518953, 0.0),
            ),
            (
                glam::DVec3::new(6642925.238247002, 5903029.863253074, -2232073.9836572986),
                glam::DVec3::new(45.400629146794955, -67.73807362936134, 0.0),
            ),
            (
                glam::DVec3::new(2591477.518994638, -5409635.6294355355, -8285118.102197416),
                glam::DVec3::new(19.970663213164315, -93.2950875704390, 0.0),
            ),
            (
                glam::DVec3::new(2528320.789589662, -3495684.1279134876, 8360309.7196244),
                glam::DVec3::new(66.11400734475339, -95.08388981259206, 0.0),
            ),
            (
                glam::DVec3::new(-9735848.884452593, -8556065.641175192, 8333421.846976527),
                glam::DVec3::new(-14.691539720606187, -2.33006016905326, 0.0),
            ),
            (
                glam::DVec3::new(7426961.41892605, 1655943.456426185, 6591322.351593178),
                glam::DVec3::new(-15.910515256034472, 165.4827913906837, 0.0),
            ),
            (
                glam::DVec3::new(-3196405.2009189655, -9604287.523781441, -1060713.7649627458),
                glam::DVec3::new(-27.470280603087133, 102.2454141837, 0.0),
            ),
            (
                glam::DVec3::new(7335138.4135223925, 2344465.657677945, 8588377.836181395),
                glam::DVec3::new(-10.5382655567836, -167.02758187958835, 0.0),
            ),
            (
                glam::DVec3::new(7310959.32431601, 8350601.114886332, 6569499.540019801),
                glam::DVec3::new(63.317884611786326, 23.40930303068805, 0.0),
            ),
            (
                glam::DVec3::new(-8640546.064591233, -2663666.0467856065, 1614399.1752727698),
                glam::DVec3::new(72.66364577543774, 163.0609809895735, 0.0),
            ),
            (
                glam::DVec3::new(-8068883.573257133, -1893830.0686977436, 8266480.72264607),
                glam::DVec3::new(-25.23522156265527, 125.3010823588916, 0.0),
            ),
            (
                glam::DVec3::new(-8234811.869950183, -3807444.077073443, 462885.92168885097),
                glam::DVec3::new(-64.46261342414945, 130.7529101785710, 0.0),
            ),
            (
                glam::DVec3::new(5745089.68100643, 8186227.487702135, 1050884.6330531444),
                glam::DVec3::new(-64.03982768384915, 2.6480032202244956, 0.0),
            ),
            (
                glam::DVec3::new(2393154.1929320674, 2067816.501552973, 3732474.8825010224),
                glam::DVec3::new(-60.74611032083097, 30.92056758445699, 0.0),
            ),
            (
                glam::DVec3::new(1951197.9768817485, -7575128.269719007, 9939982.202217888),
                glam::DVec3::new(42.50213359486636, 113.81499658197713, 0.0),
            ),
            (
                glam::DVec3::new(6216696.544619927, -2057542.5103991907, 1143021.5607241336),
                glam::DVec3::new(-30.03750128679215, -110.032118791896, 0.0),
            ),
            (
                glam::DVec3::new(8262888.4250550615, 3106595.2889045794, 9812866.097362377),
                glam::DVec3::new(15.123933131017566, 164.52989386282826, 0.0),
            ),
            (
                glam::DVec3::new(7218924.780765143, -447912.04118075967, 5171595.8773390055),
                glam::DVec3::new(81.52927937810878, -105.2611885755099, 0.0),
            ),
            (
                glam::DVec3::new(-912999.7792285755, 2374762.7068727836, -4955536.282540757),
                glam::DVec3::new(10.831088217119586, 151.2254067023637, 0.0),
            ),
            (
                glam::DVec3::new(3623182.097599946, -124808.5784964189, 6830241.830351334),
                glam::DVec3::new(71.11579335672388, -144.99222808538826, 0.0),
            ),
            (
                glam::DVec3::new(2208932.904275298, -3444783.5638758554, -7103490.483277793),
                glam::DVec3::new(22.462213216531538, -26.6324481797208, 0.0),
            ),
            (
                glam::DVec3::new(698713.3718823884, -8409581.988125127, -3903578.000227963),
                glam::DVec3::new(-87.47177257040715, -13.04761510545472, 0.0),
            ),
            (
                glam::DVec3::new(-9556270.567275014, -7440669.805674921, 2815381.5153284855),
                glam::DVec3::new(7.966991562503196, 100.79184677036972, 0.0),
            ),
            (
                glam::DVec3::new(499026.44389439, -1925421.3250966482, 7925563.273168109),
                glam::DVec3::new(-71.1002119000218, -138.06002686196018, 0.0),
            ),
            (
                glam::DVec3::new(7238450.744717773, -4383306.046812624, -8262036.304761575),
                glam::DVec3::new(-13.118964393782036, 74.96561856267363, 0.0),
            ),
            (
                glam::DVec3::new(-326376.61228840984, 7739298.466333255, -5871984.449330825),
                glam::DVec3::new(-72.49265123799492, 91.02269534514028, 0.0),
            ),
            (
                glam::DVec3::new(-1226800.3167807497, 7604616.1196159385, 244802.38239366747),
                glam::DVec3::new(45.976409115548705, -86.172644141553, 0.0),
            ),
            (
                glam::DVec3::new(-6183006.182703774, 1260934.9898499493, 5405250.482727632),
                glam::DVec3::new(-46.5078875763827, 96.5794699892873, 0.0),
            ),
            (
                glam::DVec3::new(4892851.839336393, -4366090.555875951, -284396.9303730335),
                glam::DVec3::new(-81.70294712853254, -160.1769692625534, 0.0),
            ),
            (
                glam::DVec3::new(9237967.029159822, 5403631.481081279, -3005393.284370361),
                glam::DVec3::new(-79.40102319570062, 38.14134828047784, 0.0),
            ),
            (
                glam::DVec3::new(-6538766.213631488, -4940418.412632655, 2501053.290733097),
                glam::DVec3::new(13.527045111462556, 102.20616898781418, 0.0),
            ),
            (
                glam::DVec3::new(-8784252.36675533, -8232153.597217414, 4884503.641264852),
                glam::DVec3::new(30.28635667737656, -169.57611510228907, 0.0),
            ),
            (
                glam::DVec3::new(-4943652.169548029, -3855871.065791603, -4748617.896841602),
                glam::DVec3::new(74.73754290271745, 152.61923201200796, 0.0),
            ),
            (
                glam::DVec3::new(2469337.744048135, -9190045.568734694, 4837736.320298016),
                glam::DVec3::new(-12.948807601309923, 46.97162855809245, 0.0),
            ),
            (
                glam::DVec3::new(-1433624.4808392152, -182826.48301463015, -5022036.343881993),
                glam::DVec3::new(-45.74760712894212, 152.37356328703, 0.0),
            ),
            (
                glam::DVec3::new(5344321.103997834, 2878251.6468978976, -2881088.089473922),
                glam::DVec3::new(30.33833424064295, -69.68258737927425, 0.0),
            ),
            (
                glam::DVec3::new(6606169.345653623, 322403.8278391063, 1042680.5917547438),
                glam::DVec3::new(62.631186380765996, -139.94615572408208, 0.0),
            ),
            (
                glam::DVec3::new(667958.4714587647, 8478848.064964425, 1694820.8740469385),
                glam::DVec3::new(-78.2754170386466, -114.12583810194599, 0.0),
            ),
            (
                glam::DVec3::new(8174259.422747154, 2536147.3532497776, -9862214.372994114),
                glam::DVec3::new(71.51999174030132, -146.96589979434754, 0.0),
            ),
            (
                glam::DVec3::new(-3472815.7693284587, 4576504.523164628, 5061670.1162975095),
                glam::DVec3::new(-2.043219531494401, 8.809342235384065, 0.0),
            ),
            (
                glam::DVec3::new(4164265.524093671, -8876967.849787056, 5611143.587449348),
                glam::DVec3::new(-77.72886207380276, -58.502589304409916, 0.0),
            ),
            (
                glam::DVec3::new(9676345.130538844, -6880885.856225148, 2825292.9134078994),
                glam::DVec3::new(18.694870013985096, 74.11457134859293, 0.0),
            ),
            (
                glam::DVec3::new(8345975.870587062, 5602669.236328628, 6440366.360817695),
                glam::DVec3::new(-20.537448114795794, 80.45056404450543, 0.0),
            ),
            (
                glam::DVec3::new(76565.8634340372, -5176255.943787364, 8534384.933999725),
                glam::DVec3::new(4.390777736040391, -163.5117863846071, 0.0),
            ),
            (
                glam::DVec3::new(-8621776.079462763, 8582232.035568487, -7881480.052577838),
                glam::DVec3::new(70.39862590743721, -50.56334026059545, 0.0),
            ),
            (
                glam::DVec3::new(-4399316.195656607, -6898512.909927797, 4952200.285973493),
                glam::DVec3::new(13.452784771177136, -80.37984986897065, 0.0),
            ),
        ];
        let enu: Vec<glam::DVec3> = vec![
            glam::DVec3::new(-2509734.058866111, -1142229.8051993023, -8710086.829399489),
            glam::DVec3::new(-6213939.181029341, -356139.6129727494, -4553234.7863042625),
            glam::DVec3::new(646493.0548110288, -6565953.849297021, 5657224.674040573),
            glam::DVec3::new(-8310221.5498587, -11824484.096495166, 2477362.99531654),
            glam::DVec3::new(1442851.8614025046, 3944020.582333587, -8588097.243385006),
            glam::DVec3::new(8384083.9597701, 530705.873201302, -3658122.3308675354),
            glam::DVec3::new(2898130.522006808, -9580587.360908013, 2106248.00047252),
            glam::DVec3::new(2828141.7883684454, 406381.76914518373, 8963451.345762655),
            glam::DVec3::new(-8944812.308699876, 5682066.860649477, -11186750.260459974),
            glam::DVec3::new(-3464795.3083847975, 4481617.34349062, -8322123.193586318),
            glam::DVec3::new(5160745.024628614, -4957933.438608909, -7236716.489417127),
            glam::DVec3::new(-638024.4383018238, 7039962.2571193, -9115528.775503935),
            glam::DVec3::new(4758642.742106677, -6009149.6767303115, 10372387.808200765),
            glam::DVec3::new(5065559.227112598, -6668312.075854795, 3772818.7838625945),
            glam::DVec3::new(7679624.4540795535, 8806536.758325476, -704596.7317034043),
            glam::DVec3::new(8723624.845966201, 2447560.2355613476, 656383.0818598685),
            glam::DVec3::new(7912063.637985037, 5959948.937075286, 1732914.3346207177),
            glam::DVec3::new(544218.9974970771, 4542220.806800395, -1733919.4470265843),
            glam::DVec3::new(1273660.08066382, 12542690.992105417, 1025514.8480509501),
            glam::DVec3::new(6545395.75730307, 891176.5148664078, -742222.4719711064),
            glam::DVec3::new(-5198048.819521416, 11334531.338719131, -4327503.698459759),
            glam::DVec3::new(7082255.427083047, 2213827.8816959755, 4898931.340738286),
            glam::DVec3::new(-1642041.3457062577, -5232446.364934714, 977553.893898297),
            glam::DVec3::new(2180801.9347610455, 4950825.7295749895, 5525274.352594351),
            glam::DVec3::new(-2089105.6099524165, -7908981.413461881, 537723.1402286584),
            glam::DVec3::new(-8034727.96923753, 2404524.236063574, 4013552.0992562943),
            glam::DVec3::new(10780459.878230758, 3553258.2532735206, -5076247.431745518),
            glam::DVec3::new(1765741.0366824528, 3433493.892317093, -7201673.843672094),
            glam::DVec3::new(-8127704.602222981, -8581068.9141233, -418878.4728269605),
            glam::DVec3::new(188190.11501304168, 5618723.277314343, 7929564.744095197),
            glam::DVec3::new(-716453.8452096153, 5684941.331412287, -5153958.6238624565),
            glam::DVec3::new(5997804.751024207, 5142896.894642161, -2571615.2481815317),
            glam::DVec3::new(5766620.612909903, -3130671.465188509, -169145.9772780796),
            glam::DVec3::new(-1455502.2937485129, 9869173.857722817, 4904342.692741647),
            glam::DVec3::new(7435498.038448695, 3237763.4321128717, -2765632.3199936696),
            glam::DVec3::new(6506963.899765746, -890288.4638884775, 11209666.612828782),
            glam::DVec3::new(5697491.384008396, -3774239.402854111, -3892366.7913987376),
            glam::DVec3::new(-8076048.557717319, 3586898.887116836, -5989144.782130713),
            glam::DVec3::new(826761.0616518205, -2655410.2107246453, 4424343.6206899965),
            glam::DVec3::new(6011204.366063954, -2060480.221598107, -2183257.3166331653),
            glam::DVec3::new(4004336.8956559882, 5154200.514421189, -1494022.607362235),
            glam::DVec3::new(-2856049.652164667, -7499693.903157335, -3287403.620908837),
            glam::DVec3::new(2329930.4291646713, 4684682.772947516, -11964071.395589357),
            glam::DVec3::new(5054368.641846909, 4961083.740458888, -2909699.8362255576),
            glam::DVec3::new(-1087142.537981118, 10714673.301033907, -3411816.9755286165),
            glam::DVec3::new(-11190213.783338673, 3948581.5711958185, -2854530.496779622),
            glam::DVec3::new(-7300849.649118861, 8455065.135673758, 4211057.18533386),
            glam::DVec3::new(4985129.447896278, 8402484.675946508, 2044981.6104112952),
            glam::DVec3::new(-1207186.325554382, 8759529.854828134, -11485677.829091577),
            glam::DVec3::new(-5490298.462615342, 3405031.6482858593, 7051959.535886337),
        ];
        return (ecef_ref_tuple, enu);
    }
    #[fixture]
    fn ecef2ned_fixture() -> (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>) {
        let ecef_ref_tuple: Vec<(glam::DVec3, glam::DVec3)> = vec![
            (
                glam::DVec3::new(6888437.030500963, 5159088.058806049, -1588568.383383099),
                glam::DVec3::new(-43.39498494726661, 4.058899692699072, 0.0),
            ),
            (
                glam::DVec3::new(5675971.780695453, -3933745.478421451, -468060.9169528838),
                glam::DVec3::new(15.008767101905619, 146.92063867032067, 0.0),
            ),
            (
                glam::DVec3::new(-4363243.112005923, 5116084.0831444785, 2367379.933506632),
                glam::DVec3::new(-44.90885855476071, 147.50865214856645, 0.0),
            ),
            (
                glam::DVec3::new(6204344.719931791, 8043319.008791654, -3797048.613613347),
                glam::DVec3::new(41.36971468682316, 143.58178366847767, 0.0),
            ),
            (
                glam::DVec3::new(-557145.6909457333, -7985975.838632684, -1316563.2909243256),
                glam::DVec3::new(19.959655219884283, 148.68397916564334, 0.0),
            ),
            (
                glam::DVec3::new(-459804.4689456597, 7306198.5554328, -4790153.792160811),
                glam::DVec3::new(54.905008862344026, 17.53174938081216, 0.0),
            ),
            (
                glam::DVec3::new(4394093.728079082, -2023529.1555146254, 6496899.54296466),
                glam::DVec3::new(30.26757622173315, -179.5885850468058, 0.0),
            ),
            (
                glam::DVec3::new(7352055.509855617, -5121782.462257361, -3495912.7450521984),
                glam::DVec3::new(66.68482177955784, -111.21584705913939, 0.0),
            ),
            (
                glam::DVec3::new(-5227681.427695597, 9350805.005802866, 6063589.3855974),
                glam::DVec3::new(-9.365477141597339, -151.03950532108723, 0.0),
            ),
            (
                glam::DVec3::new(158812.85041147843, 8656676.484538134, -7818843.081377927),
                glam::DVec3::new(9.228104296299222, 74.36210755208026, 0.0),
            ),
            (
                glam::DVec3::new(6289337.265826721, 805672.1394064799, 9276770.919476017),
                glam::DVec3::new(18.573413033048936, 31.542143103157088, 0.0),
            ),
            (
                glam::DVec3::new(1925737.2316621258, -2301977.0805467907, 1513020.2832977697),
                glam::DVec3::new(-37.74068956750355, -111.81912172043178, 0.0),
            ),
            (
                glam::DVec3::new(2255463.5973721338, 3133187.779792577, -469380.1598123852),
                glam::DVec3::new(-73.83161498479313, 92.73741190791725, 0.0),
            ),
            (
                glam::DVec3::new(8467620.318925612, 6849204.462803648, 7963462.42715758),
                glam::DVec3::new(76.15483916763182, 14.615972981299592, 0.0),
            ),
            (
                glam::DVec3::new(4105667.997088125, -4487317.573757457, 6232574.17015757),
                glam::DVec3::new(62.907473733546084, 142.21402827360305, 0.0),
            ),
            (
                glam::DVec3::new(8995297.46464241, 1593900.2149121184, -988737.8673768956),
                glam::DVec3::new(28.84416815203001, 178.65282216728616, 0.0),
            ),
            (
                glam::DVec3::new(5866501.682604484, -8352540.236067053, 2255662.100814244),
                glam::DVec3::new(-2.440043645549977, 46.85304254813022, 0.0),
            ),
            (
                glam::DVec3::new(-5139287.558762874, 4629784.415816955, -7657314.13582964),
                glam::DVec3::new(-50.317103363790864, 106.04986981580731, 0.0),
            ),
            (
                glam::DVec3::new(6318261.930673189, -7987849.595678076, -7072830.221753922),
                glam::DVec3::new(35.58071523442298, -163.71573556837956, 0.0),
            ),
            (
                glam::DVec3::new(8200320.293980793, 683959.3652144801, 3611782.65124513),
                glam::DVec3::new(-85.19457696080306, 48.59996756812498, 0.0),
            ),
            (
                glam::DVec3::new(1519058.9606308155, -2175811.8135434627, -2597201.19329625),
                glam::DVec3::new(86.49299711650835, -166.89886645986513, 0.0),
            ),
            (
                glam::DVec3::new(9220625.604792222, -6300561.172051234, -7522096.711511366),
                glam::DVec3::new(-52.096228220403646, 108.26877252750512, 0.0),
            ),
            (
                glam::DVec3::new(-9544348.486626834, -1487623.3606636561, -7969995.612516604),
                glam::DVec3::new(-43.21441983729025, -100.50146232612576, 0.0),
            ),
            (
                glam::DVec3::new(-2994120.6520693544, -6393641.969406243, 72730.10419774428),
                glam::DVec3::new(-82.91183272475539, -143.668353171972, 0.0),
            ),
            (
                glam::DVec3::new(-6012884.190658741, -2828893.973767963, 4631966.124507211),
                glam::DVec3::new(60.89878173481494, 150.6535423183193, 0.0),
            ),
            (
                glam::DVec3::new(3452811.2714610514, 9330978.060863663, -8838981.123470027),
                glam::DVec3::new(31.71632117388809, 124.35285373258189, 0.0),
            ),
            (
                glam::DVec3::new(-4986253.214297766, 1935827.8693882204, -1153719.326018421),
                glam::DVec3::new(-58.5324927987406, -10.21485056533632, 0.0),
            ),
            (
                glam::DVec3::new(1382254.7904856037, 172002.60125266388, -3771079.9799958635),
                glam::DVec3::new(-25.712697133752684, 121.55802277283243, 0.0),
            ),
            (
                glam::DVec3::new(1212004.377070479, -9751273.623413712, 4831487.548213271),
                glam::DVec3::new(-29.535020194777093, -163.54926231537002, 0.0),
            ),
            (
                glam::DVec3::new(-5197391.84347292, 9062586.796555977, -2955488.769689851),
                glam::DVec3::new(-38.181975325847986, -50.68756898865132, 0.0),
            ),
            (
                glam::DVec3::new(2674957.0449850517, 2421536.9123733453, 4312387.006029125),
                glam::DVec3::new(-20.156897643748977, -30.80952422019098, 0.0),
            ),
            (
                glam::DVec3::new(-9969515.562865596, -6153809.175106484, -3311966.1867499687),
                glam::DVec3::new(-46.90512716652745, 49.463784406548086, 0.0),
            ),
            (
                glam::DVec3::new(7508467.8342603445, 1363028.4182038382, -1711872.0663271137),
                glam::DVec3::new(-17.591926478565682, 72.65866461612316, 0.0),
            ),
            (
                glam::DVec3::new(3243917.7794763483, -9064406.280864034, -1092956.2056234032),
                glam::DVec3::new(-43.339153779499895, -123.2328340359681, 0.0),
            ),
            (
                glam::DVec3::new(-254687.97861935943, 1228098.5122885387, 5109695.345173651),
                glam::DVec3::new(69.09752776476617, -1.9502386648967445, 0.0),
            ),
            (
                glam::DVec3::new(-662155.2929495294, 6180917.147207247, 7500326.629605424),
                glam::DVec3::new(56.234687825476634, -112.31953414170192, 0.0),
            ),
            (
                glam::DVec3::new(2661775.1983660073, -8330658.996485413, 4511087.109226249),
                glam::DVec3::new(87.62786643692306, -35.345944003484306, 0.0),
            ),
            (
                glam::DVec3::new(-3676457.2555731535, -5729506.758706078, 4346482.866220744),
                glam::DVec3::new(-89.5756383505163, 116.18330779130967, 0.0),
            ),
            (
                glam::DVec3::new(-8044313.163986813, -7621922.104305084, 2985308.497923072),
                glam::DVec3::new(67.25768830206161, -79.20621240232589, 0.0),
            ),
            (
                glam::DVec3::new(-7996386.218725819, 7078762.191946764, -2066076.453381911),
                glam::DVec3::new(-75.35782498171784, -81.10301636906564, 0.0),
            ),
            (
                glam::DVec3::new(5846830.623713044, 7227198.072744723, -7331588.91594902),
                glam::DVec3::new(3.755795114555795, 54.281965733905416, 0.0),
            ),
            (
                glam::DVec3::new(7437276.714211722, -4431803.69567279, -9628513.449088097),
                glam::DVec3::new(-82.68061073845303, 65.15883724004757, 0.0),
            ),
            (
                glam::DVec3::new(8930051.083399918, 8768775.994698372, 8197023.54810205),
                glam::DVec3::new(-82.43918424587858, 89.6885364207107, 0.0),
            ),
            (
                glam::DVec3::new(3107237.2934945915, 4247153.050324835, 8054203.0123866135),
                glam::DVec3::new(25.225415962780346, -45.91826532998783, 0.0),
            ),
            (
                glam::DVec3::new(-5843117.926183505, 1742510.0939028692, -9822058.359018423),
                glam::DVec3::new(-62.81582870448219, -59.9729803092481, 0.0),
            ),
            (
                glam::DVec3::new(4369988.455430791, -3234880.5994664277, 2410762.1663310328),
                glam::DVec3::new(-82.58346908888234, -121.01020355679265, 0.0),
            ),
            (
                glam::DVec3::new(-4209382.927282661, -2104160.3402341865, 969685.9314502683),
                glam::DVec3::new(-37.18673973767942, -7.896719105632457, 0.0),
            ),
            (
                glam::DVec3::new(-9034872.754234113, -6408263.019168887, 461004.63400196284),
                glam::DVec3::new(-77.24468086301746, -34.859107279766334, 0.0),
            ),
            (
                glam::DVec3::new(-1705567.8205711525, -8011993.235225978, 8173151.087935612),
                glam::DVec3::new(-4.6791627952866435, 122.70539974596176, 0.0),
            ),
            (
                glam::DVec3::new(-3126968.1268446445, -418269.6169602778, 3991905.823012369),
                glam::DVec3::new(-13.22364176207492, -71.31487816103186, 0.0),
            ),
        ];
        let ned: Vec<glam::DVec3> = vec![
            glam::DVec3::new(3817222.48054082, 4658571.265151352, -6349553.538000638),
            glam::DVec3::new(1335560.479667212, 198199.46328847948, 6788746.964214704),
            glam::DVec3::new(6215046.4463947285, -1971463.5162006821, -2881531.5065581324),
            glam::DVec3::new(-2705751.452139706, -10155862.596903786, 2672791.8450811454),
            glam::DVec3::new(16941.438273651525, 7112108.151736968, 3903484.4614574388),
            glam::DVec3::new(-4196041.589283414, 7105335.751442247, 2906022.9995483346),
            glam::DVec3::new(7818661.838767107, 2055028.707839415, 507757.48807762563),
            glam::DVec3::new(-3325079.223590582, 8707243.769614134, 2373717.722279952),
            glam::DVec3::new(5990291.986467549, -10712799.23411021, 941094.0918214218),
            glam::DVec3::new(-9061359.615151051, 2180531.241850624, -7016738.689651038),
            glam::DVec3::new(6952053.224228817, -2603474.54049187, -8435296.427302884),
            glam::DVec3::new(2066450.2991287007, 2643374.4640652123, -197856.88363498566),
            glam::DVec3::new(2771666.85982101, -2402526.7977076243, -1292295.3383048982),
            glam::DVec3::new(-7728001.629512573, 4490844.479939028, -10106395.803587643),
            glam::DVec3::new(8174937.185333731, 1030751.23863034, -2818769.837445282),
            glam::DVec3::new(3454244.953396713, -1804943.7863353621, 8321289.964481875),
            glam::DVec3::new(2164974.005012828, -9992279.672711134, 2176239.0493952204),
            glam::DVec3::new(-371837.78150431626, 3658950.7480725506, -9641340.996500337),
            glam::DVec3::new(-3526752.99639549, 9439055.971421905, 7226156.516731704),
            glam::DVec3::new(6217719.1737010805, -5698837.223374292, 3101813.958870949),
            glam::DVec3::new(825608.3386346335, 2463504.6292084446, 2652671.676882206),
            glam::DVec3::new(-11622623.652663339, -6780805.413641624, -483987.5357060544),
            glam::DVec3::new(-3615814.874120512, -9113347.937575273, -7791098.351688512),
            glam::DVec3::new(6161623.468428223, 3376833.985513638, -692888.337145604),
            glam::DVec3::new(-1115448.4565121694, 5412718.962804998, -5922058.40516899),
            glam::DVec3::new(-10544501.6812377, -8115923.044843912, -248841.26316541713),
            glam::DVec3::new(-5080617.0732847275, 1020883.0769152861, 1756800.1702353738),
            glam::DVec3::new(-3647944.2084103096, -1267853.540811822, -1116381.595853447),
            glam::DVec3::new(4991934.067786062, 9695319.888885854, 990419.2709722328),
            glam::DVec3::new(-8693038.257175997, 1720354.0762511175, 6272937.350163244),
            glam::DVec3::new(4412558.882047785, 3449871.5464713047, 493587.2015961539),
            glam::DVec3::new(-10409383.191861024, 3577248.750890754, 5203629.587439036),
            glam::DVec3::new(-562179.2525078077, -6760908.88330678, -3890951.968104692),
            glam::DVec3::new(3188546.913593634, 7681057.538208658, -4971472.691921508),
            glam::DVec3::new(2099860.9212028678, 1218719.7314506723, -4667695.873833862),
            glam::DVec3::new(8712939.198567996, -2959883.4468468903, -3197008.9875583653),
            glam::DVec3::new(-6797822.957480535, -5255233.032677077, -4796557.332128665),
            glam::DVec3::new(-3487079.84736866, 5827317.746543763, 4372429.642127713),
            glam::DVec3::new(-4361519.839891313, -9329383.645262405, -5065222.912952845),
            glam::DVec3::new(-8485272.391840788, -6805383.920264025, 81490.94709643815),
            glam::DVec3::new(-7923794.00309282, -527825.6411444983, -8780939.872624164),
            glam::DVec3::new(-2116687.4840525235, -8610968.474886889, -9435735.063012568),
            glam::DVec3::new(9819083.582509633, -8882251.737492235, 6965604.174583564),
            glam::DVec3::new(7665132.586816506, 5186753.740633406, -2628069.0356712104),
            glam::DVec3::new(-8430208.39139448, -4186943.6469871406, -6712100.1414773455),
            glam::DVec3::new(827984.89422318, 5411990.781832159, 2323321.460391936),
            glam::DVec3::new(-1572839.8441506187, -2662525.2591259694, 3677474.7097341423),
            glam::DVec3::new(-3556595.327789179, -10422339.65689887, 1277791.7010225418),
            glam::DVec3::new(7671120.493029718, 5764204.045919783, 6467554.251532741),
            glam::DVec3::new(3747536.9393992727, -3096156.3708752114, 1502655.6004291987),
        ];
        return (ecef_ref_tuple, ned);
    }

    #[rstest]
    fn test_ecef2lla_ferarri(ecef_fixture: (Vec<glam::DVec3>, Vec<glam::DVec3>)) {
        let ecefs = ecef_fixture.0;
        let llas = ecef_fixture.1;
        for (i, ecef) in ecefs.iter().enumerate() {
            let actual_lla: glam::DVec3 = ecef2lla_ferarri(ecef);
            let expected_lla = llas.get(i).unwrap();
            assert_vecs_close(&actual_lla, expected_lla, 1e-6);
        }
    }
    #[rstest]
    fn test_ecef2lla_map3d(ecef_fixture: (Vec<glam::DVec3>, Vec<glam::DVec3>)) {
        let ecefs = ecef_fixture.0;
        let llas = ecef_fixture.1;
        for (i, ecef) in ecefs.iter().enumerate() {
            let actual_lla: glam::DVec3 = ecef2lla_map3d(ecef);
            let expected_lla = llas.get(i).unwrap();
            assert_vecs_close(&actual_lla, expected_lla, 1e-6);
        }
    }

    #[test]
    fn test_ecef2lla_zero_map3() {
        let zero = glam::DVec3::new(0., 0., 0.);
        let actual = ecef2lla_map3d(&zero);
        assert!(almost::zero(actual.x));
        assert!(almost::zero(actual.y));
        assert!(almost::equal_with(actual.z, -6378137.0, 1e-6));
    }
    #[test]
    fn test_ecef2lla_zero_ferrari() {
        let zero = glam::DVec3::new(0., 0., 0.);
        let actual = ecef2lla_ferarri(&zero);
        assert!(almost::zero(actual.x));
        assert!(almost::zero(actual.y));
        assert!(almost::equal_with(actual.z, -6292741.654585736, 1e-6));
    }

    #[rstest]
    fn test_lla2ecef(lla_fixture: (Vec<glam::DVec3>, Vec<glam::DVec3>)) {
        let llas = lla_fixture.0;
        let ecefs = lla_fixture.1;
        for (i, lla) in llas.iter().enumerate() {
            let actual_ecef = lla2ecef(lla);
            let expected_ecef = ecefs.get(i).unwrap();
            assert_vecs_close(&actual_ecef, expected_ecef, 1e-6);
        }
    }

    #[rstest]
    fn test_ecef2enu(ecef2enu_fixture: (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>)) {
        let ecef_ref_tuples = ecef2enu_fixture.0;
        let enu = ecef2enu_fixture.1;
        for (i, ecef_ref_tuple) in ecef_ref_tuples.iter().enumerate() {
            let actual_enu = ecef2enu(&ecef_ref_tuple.0, &ecef_ref_tuple.1);
            let expected_enu = enu.get(i).unwrap();
            assert_vecs_close(&actual_enu, expected_enu, 1e-6);
        }
    }
    #[rstest]
    fn test_enu2ecef(ecef2enu_fixture: (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>)) {
        let ecef_ref_tuples = ecef2enu_fixture.0;
        let enu = ecef2enu_fixture.1;
        for (i, ecef_ref_tuple) in ecef_ref_tuples.iter().enumerate() {
            let actual_ecef = enu2ecef(&enu.get(i).unwrap(), &ecef_ref_tuple.1);
            let expected_ecef = ecef_ref_tuple.0;
            assert_vecs_close(&actual_ecef, &expected_ecef, 1e-6);
        }
    }

    #[rstest]
    fn test_ecef2ned_dcm() {
        let actual = ecef2ned_dcm(0., 0.);
        let ned = actual * glam::DVec3::new(0.5, -1., 1.);
        assert_vecs_close(&ned, &glam::DVec3::new(1., -1., -0.5), 1e-6);
    }
    #[rstest]
    fn test_ecef2ned(ecef2ned_fixture: (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>)) {
        let ecef_ref_tuples = ecef2ned_fixture.0;
        let neds = ecef2ned_fixture.1;
        for (i, ecef_ref_tuple) in ecef_ref_tuples.iter().enumerate() {
            let actual_ned = ecef2ned(&ecef_ref_tuple.0, &ecef_ref_tuple.1);
            let expected_ned = neds.get(i).unwrap();
            assert_vecs_close(&actual_ned, expected_ned, 1e-6);
        }
    }
    #[rstest]
    fn test_ned2ecef(ecef2ned_fixture: (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>)) {
        let ecef_ref_tuples = ecef2ned_fixture.0;
        let neds = ecef2ned_fixture.1;
        for (i, ecef_ref_tuple) in ecef_ref_tuples.iter().enumerate() {
            let actual_ecef = ned2ecef(&neds.get(i).unwrap(), &ecef_ref_tuple.1);
            let expected_ecef = ecef_ref_tuple.0;
            assert_vecs_close(&actual_ecef, &expected_ecef, 1e-6);
        }
    }

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

    #[rstest]
    fn test_enu2heading() {
        {
            let actual_heading = enu2heading(&glam::DVec3::new(1000., 1000., 0.));
            let expected_heading = 45.;
            assert!(almost::equal_with(actual_heading, expected_heading, 1e-10));
        }
        {
            let actual_heading = enu2heading(&glam::DVec3::new(1000., -1000., 0.));
            let expected_heading = 135.;
            assert!(almost::equal_with(actual_heading, expected_heading, 1e-10));
        }
    }

    #[test]
    fn test_dms_lower2dd() {
        let lat_dms = "25:22:44.738n";
        let lon_dms = "74:59:55.525w";
        let lat = dms2dd(&lat_dms);
        let lon = dms2dd(&lon_dms);

        assert!(lat.is_ok());
        assert!(lon.is_ok());
        assert!(almost::equal_with(lat.unwrap(), 25.379094, 1e-6));
        assert!(almost::equal_with(lon.unwrap(), -74.998757, 1e-6));
    }
    #[test]
    fn test_dms_upper2dd() {
        let lat_dms = "25:22:44.738N";
        let lon_dms = "74:59:55.525W";
        let lat = dms2dd(&lat_dms);
        let lon = dms2dd(&lon_dms);

        assert!(lat.is_ok());
        assert!(lon.is_ok());
        assert!(almost::equal_with(lat.unwrap(), 25.379094, 1e-6));
        assert!(almost::equal_with(lon.unwrap(), -74.998757, 1e-6));
    }
    #[test]
    fn test_dms2dd_throws_err_on_bad_cardinal() {
        let lat_dms = "25:22:44.738R";
        let lat = dms2dd(&lat_dms);

        assert!(lat.is_err());
    }
    #[test]
    fn test_dms2dd_int_sec() {
        let lat_dms = "25:22:44N";
        let lat = dms2dd(&lat_dms);

        assert!(lat.is_ok());
    }
    #[test]
    fn test_dms2dd_throws_err_on_bad_min() {
        let lat_dms = "25:2244N";
        let lat = dms2dd(&lat_dms);

        assert!(lat.is_err());
    }

    #[test]
    fn test_dd2dms() {
        let lat_dd = 25.379094;
        let lon_dd = -74.998757;
        let lat_dms = dd2dms(lat_dd, true);
        let lon_dms = dd2dms(lon_dd, false);
        assert!(lat_dms == "25:22:44.738N");
        assert!(lon_dms == "74:59:55.525W");
    }

    #[rstest]
    #[case((45.0, 120.0, 0.0, 0.0), (45.0, 120.0))] // Edge case (same location)
    #[case((45.0, 120.0, 3000.0, 30.0), (45.02337670419947, 120.0190319660863))] // Sample case 1
    #[case((-10.0, -80.0, 840000.0, -113.0), (-12.884359520148694, -87.12194168465778))] // Sample case 2
    #[case((-60.0, 24.0, 12000000.0, -45.0), (37.34976350706342, -33.49808323545486))] // Sample case 3
    fn test_vincenty_direct(#[case] args: (f64, f64, f64, f64), #[case] truth: (f64, f64)) {
        let result = vincenty_direct(args.0, args.1, args.2, args.3, 1.0E-13, 2000);
        assert!(result.is_ok());

        let test = result.unwrap();
        assert!(relative_eq!(
            test.0,
            truth.0,
            max_relative = 1e-8,
            epsilon = 1e-9
        ));
        assert!(relative_eq!(
            test.1,
            truth.1,
            max_relative = 1e-8,
            epsilon = 1e-9
        ));
    }

    #[rstest]
    #[case((45.0, 120.0, 45.0, 120.0), (0.0, 0.0, 0.0))] // Edge case | Same location
    #[case((90.0, 0.0, -90.0, 0.0), (20003931.457984865, 180.0, 0.0))] // Edge case | Anti-podal
    #[case((0.0, -10.0, 0.0, 10.0), (2226389.8158654678, 90.0, -90.0))] // Edge case | Both on equator
    #[case((-60.0, 24.0, -64.0, 26.0), (457876.09259014280, 167.65057682653136, -14.116435240448425))] // Sample case 1
    #[case((18.0, -175.0, -30.0, 150.0), (6503644.0543462737, -144.26832642124467, 39.866234335863595))] // Sample case 2
    fn test_vincenty_inverse(#[case] args: (f64, f64, f64, f64), #[case] truth: (f64, f64, f64)) {
        let result = vincenty_inverse(args.0, args.1, args.2, args.3, 1.0E-13, 2000);
        assert!(result.is_ok());

        let test = result.unwrap();
        assert!(relative_eq!(
            test.0,
            truth.0,
            max_relative = 1e-4,
            epsilon = 1e-5
        ));
        assert!(relative_eq!(
            test.1,
            truth.1,
            max_relative = 1e-10,
            epsilon = 1e-11
        ));
        assert!(relative_eq!(
            test.2,
            truth.2,
            max_relative = 1e-10,
            epsilon = 1e-11
        ));
    }
}
