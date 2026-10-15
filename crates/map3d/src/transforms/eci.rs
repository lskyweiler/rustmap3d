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

#[cfg(test)]
mod test_eci {
    use super::*;
    use rstest::*;

    #[test]
    fn test_ecef2eci_identity_at_lock() {
        let ecef = glam::DVec3::new(-4395937.137069274, -5292832.080918098, 269071.20488286763);
        assert_eq!(ecef2eci(&ecef, 0.0), ecef);
        assert_eq!(eci2ecef(&ecef, 0.0), ecef);
    }

    #[test]
    fn test_ecef2eci_rotates_eastward() {
        // a quarter of a sidereal rotation carries the prime meridian (+x) to inertial +y
        let quarter_turn_s = std::f64::consts::FRAC_PI_2 / wgs84::EARTH_ANGULAR_VEL_RADPS;
        let eci = ecef2eci(&(glam::DVec3::X * 6378137.0), quarter_turn_s);
        assert!(eci.abs_diff_eq(glam::DVec3::Y * 6378137.0, 1e-6), "{eci:?}");
    }

    #[test]
    fn test_ecef2eci_full_rotation_is_identity() {
        let sidereal_day_s = std::f64::consts::TAU / wgs84::EARTH_ANGULAR_VEL_RADPS;
        let ecef = glam::DVec3::new(1234567.0, -5432109.0, 3141592.0);
        let eci = ecef2eci(&ecef, sidereal_day_s);
        assert!(eci.abs_diff_eq(ecef, 1e-6), "{eci:?}");
    }

    #[rstest]
    #[case(0.0)]
    #[case(1.5)]
    #[case(86400.0)]
    #[case(-3600.0)]
    fn test_eci_round_trip(#[case] t: f64) {
        let ecef = glam::DVec3::new(1234567.0, -5432109.0, 3141592.0);
        let actual = eci2ecef(&ecef2eci(&ecef, t), t);
        assert!(actual.abs_diff_eq(ecef, 1e-6), "t={t}: {actual:?}");
    }
}
