use crate::constants::wgs84;
use chrono::{Datelike, NaiveDateTime, Timelike};
use glam;

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
    let rot_axis = glam::DVec3::new(0., 0., wgs84::EARTH_ANGULAR_VEL_RADPS);
    return rot_axis.cross(*earth_pos);
}

/// Converts ecef to eci assuming ecef and eci were aligned time_since_eci_lock seconds ago
/// This uses a simple constant rotation assumption for earth
pub fn ecef2eci(ecef: &glam::DVec3, time_since_eci_lock: f64) -> glam::DVec3 {
    let earth_rotation_angle: f64 = time_since_eci_lock * wgs84::EARTH_ANGULAR_VEL_RADPS;

    let eci = glam::DQuat::from_rotation_z(earth_rotation_angle) * (*ecef);
    return eci;
}

pub fn eci2ecef(eci: &glam::DVec3, time_since_eci_lock: f64) -> glam::DVec3 {
    let earth_rotation_angle: f64 = time_since_eci_lock * -wgs84::EARTH_ANGULAR_VEL_RADPS;
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
