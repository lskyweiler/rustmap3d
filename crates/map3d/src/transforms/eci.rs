use crate::{constants::wgs84, transforms::batch};
use chrono::{Datelike, NaiveDateTime, Timelike};
use glam;

/// Computes the Julian date of a UTC date and time.
///
/// Vallado, *Fundamentals of Astrodynamics and Applications*, Algorithm 14. Valid for years 1900
/// through 2100.
pub fn juliandate(year: i64, month: i64, day: i64, hour: i64, min: i64, sec: i64) -> f64 {
    // Vallado's INT() truncates toward zero, which integer division matches for these positive terms
    let day_number = 367 * year - (7 * (year + (month + 9) / 12)) / 4 + (275 * month) / 9 + day;
    let jd = day_number as f64
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
    return ecef2eci_kernel(time_since_eci_lock)(*ecef);
}

/// Per-time setup for [`ecef2eci`], shared with its batched form.
fn ecef2eci_kernel(time_since_eci_lock: f64) -> impl Fn(glam::DVec3) -> glam::DVec3 {
    let earth_rotation_angle: f64 = time_since_eci_lock * wgs84::EARTH_ANGULAR_VEL_RADPS;

    let rot = glam::DQuat::from_rotation_z(earth_rotation_angle);
    return move |ecef| rot * ecef;
}

pub fn eci2ecef(eci: &glam::DVec3, time_since_eci_lock: f64) -> glam::DVec3 {
    return eci2ecef_kernel(time_since_eci_lock)(*eci);
}

/// Per-time setup for [`eci2ecef`], shared with its batched form.
fn eci2ecef_kernel(time_since_eci_lock: f64) -> impl Fn(glam::DVec3) -> glam::DVec3 {
    let earth_rotation_angle: f64 = time_since_eci_lock * -wgs84::EARTH_ANGULAR_VEL_RADPS;
    let rot = glam::DQuat::from_rotation_z(earth_rotation_angle);
    return move |eci| rot * eci;
}

/// Converts a batch of ECEF vectors to ECI, sharing one time. Matches [`ecef2eci`] applied to each
/// point.
///
/// # Arguments
///
/// * `ecefs` - Vectors represented in ECEF coordinates [[meters]].
/// * `time_since_eci_lock` - Seconds since the ECEF and ECI frames were aligned [[seconds]].
///
/// # Returns
///
/// * Vectors represented in ECI coordinates [[meters]].
pub fn ecef2eci_batch(ecefs: &[glam::DVec3], time_since_eci_lock: f64) -> Vec<glam::DVec3> {
    return batch::map(ecefs, ecef2eci_kernel(time_since_eci_lock));
}

/// Converts a batch of ECEF vectors to ECI, sharing one time and writing into `out`. Matches
/// [`ecef2eci`] applied to each point.
///
/// # Arguments
///
/// * `ecefs` - Vectors represented in ECEF coordinates [[meters]].
/// * `time_since_eci_lock` - Seconds since the ECEF and ECI frames were aligned [[seconds]].
/// * `out` - Receives the vectors in ECI coordinates [[meters]].
///
/// # Panics
///
/// If `ecefs` and `out` differ in length.
pub fn ecef2eci_batch_into(
    ecefs: &[glam::DVec3],
    time_since_eci_lock: f64,
    out: &mut [glam::DVec3],
) {
    batch::map_into(ecefs, out, ecef2eci_kernel(time_since_eci_lock));
}

/// Converts a batch of ECI vectors to ECEF, sharing one time. Matches [`eci2ecef`] applied to each
/// point.
///
/// # Arguments
///
/// * `ecis` - Vectors represented in ECI coordinates [[meters]].
/// * `time_since_eci_lock` - Seconds since the ECEF and ECI frames were aligned [[seconds]].
///
/// # Returns
///
/// * Vectors represented in ECEF coordinates [[meters]].
pub fn eci2ecef_batch(ecis: &[glam::DVec3], time_since_eci_lock: f64) -> Vec<glam::DVec3> {
    return batch::map(ecis, eci2ecef_kernel(time_since_eci_lock));
}

/// Converts a batch of ECI vectors to ECEF, sharing one time and writing into `out`. Matches
/// [`eci2ecef`] applied to each point.
///
/// # Arguments
///
/// * `ecis` - Vectors represented in ECI coordinates [[meters]].
/// * `time_since_eci_lock` - Seconds since the ECEF and ECI frames were aligned [[seconds]].
/// * `out` - Receives the vectors in ECEF coordinates [[meters]].
///
/// # Panics
///
/// If `ecis` and `out` differ in length.
pub fn eci2ecef_batch_into(
    ecis: &[glam::DVec3],
    time_since_eci_lock: f64,
    out: &mut [glam::DVec3],
) {
    batch::map_into(ecis, out, eci2ecef_kernel(time_since_eci_lock));
}

/// Converts ECEF to an Earth-rotation-only ECI frame at a UTC time.
///
/// Rotates about the z axis by the Earth Rotation Angle (IERS Conventions 2010, eq. 5.15). This
/// does **not** apply precession, nutation, or polar motion, so the result is not the J2000/GCRF
/// frame. Precession alone moves it away from J2000 by ~0.14 degrees per decade from the epoch,
/// ~0.36 degrees by 2026: about 40 km at the equator and 270 km at geostationary range. UTC is
/// used in place of UT1, which adds up to ~0.9 s of rotation (~400 m at the equator).
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
/// * `utc_time_str` - UTC date and time.
/// * `utc_time_str_fmt` - `chrono` format of `utc_time_str`, defaults to `%Y-%m-%dT%H:%M:%S`.
///
/// # Returns
///
/// * `eci` - Vector represented in the rotated frame [[meters]].
pub fn ecef2eci_j2000(
    ecef: &glam::DVec3,
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
) -> glam::DVec3 {
    return ecef2eci_j2000_kernel(utc_time_str, utc_time_str_fmt)(*ecef);
}

/// Earth Rotation Angle at a UTC time, IERS Conventions (2010) eq. 5.15 [[radians]].
fn earth_rotation_angle(utc_time_str: String, utc_time_str_fmt: Option<String>) -> f64 {
    let jd = juliandate_from_utc_str(utc_time_str, utc_time_str_fmt);
    let tu = jd - 2451545.0;
    return std::f64::consts::TAU * (0.7790572732640 + 1.00273781191135448 * tu);
}

/// Per-time setup for [`ecef2eci_j2000`], shared with its batched form.
fn ecef2eci_j2000_kernel(
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
) -> impl Fn(glam::DVec3) -> glam::DVec3 {
    let rot = glam::DQuat::from_rotation_z(earth_rotation_angle(utc_time_str, utc_time_str_fmt));
    return move |ecef| rot * ecef;
}

/// Converts a batch of ECEF vectors to the Earth-rotation-only ECI frame, sharing one UTC time.
/// Matches [`ecef2eci_j2000`] applied to each point, with the same limitations.
///
/// # Arguments
///
/// * `ecefs` - Vectors represented in ECEF coordinates [[meters]].
/// * `utc_time_str` - UTC date and time, parsed once for the whole batch.
/// * `utc_time_str_fmt` - `chrono` format of `utc_time_str`, defaults to `%Y-%m-%dT%H:%M:%S`.
///
/// # Returns
///
/// * Vectors represented in the rotated frame [[meters]].
pub fn ecef2eci_j2000_batch(
    ecefs: &[glam::DVec3],
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
) -> Vec<glam::DVec3> {
    return batch::map(ecefs, ecef2eci_j2000_kernel(utc_time_str, utc_time_str_fmt));
}

/// Converts a batch of ECEF vectors to the Earth-rotation-only ECI frame, sharing one UTC time and
/// writing into `out`. Matches [`ecef2eci_j2000`] applied to each point.
///
/// # Arguments
///
/// * `ecefs` - Vectors represented in ECEF coordinates [[meters]].
/// * `utc_time_str` - UTC date and time, parsed once for the whole batch.
/// * `utc_time_str_fmt` - `chrono` format of `utc_time_str`, defaults to `%Y-%m-%dT%H:%M:%S`.
/// * `out` - Receives the vectors in the rotated frame [[meters]].
///
/// # Panics
///
/// If `ecefs` and `out` differ in length.
pub fn ecef2eci_j2000_batch_into(
    ecefs: &[glam::DVec3],
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
    out: &mut [glam::DVec3],
) {
    batch::map_into(
        ecefs,
        out,
        ecef2eci_j2000_kernel(utc_time_str, utc_time_str_fmt),
    );
}

/// Converts from the Earth-rotation-only ECI frame of [`ecef2eci_j2000`] back to ECEF.
///
/// Inverse of [`ecef2eci_j2000`], with the same limitations: no precession, nutation, or polar
/// motion, and UTC in place of UT1.
///
/// # Arguments
///
/// * `eci` - Vector represented in the rotated frame [[meters]].
/// * `utc_time_str` - UTC date and time.
/// * `utc_time_str_fmt` - `chrono` format of `utc_time_str`, defaults to `%Y-%m-%dT%H:%M:%S`.
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
pub fn eci2ecef_j2000(
    eci: &glam::DVec3,
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
) -> glam::DVec3 {
    return eci2ecef_j2000_kernel(utc_time_str, utc_time_str_fmt)(*eci);
}

/// Per-time setup for [`eci2ecef_j2000`], shared with its batched form.
fn eci2ecef_j2000_kernel(
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
) -> impl Fn(glam::DVec3) -> glam::DVec3 {
    let rot = glam::DMat3::from_rotation_z(-earth_rotation_angle(utc_time_str, utc_time_str_fmt));
    return move |eci| rot * eci;
}

/// Converts a batch of vectors from the Earth-rotation-only ECI frame to ECEF, sharing one UTC
/// time. Matches [`eci2ecef_j2000`] applied to each point, with the same limitations.
///
/// # Arguments
///
/// * `ecis` - Vectors represented in the rotated frame [[meters]].
/// * `utc_time_str` - UTC date and time, parsed once for the whole batch.
/// * `utc_time_str_fmt` - `chrono` format of `utc_time_str`, defaults to `%Y-%m-%dT%H:%M:%S`.
///
/// # Returns
///
/// * Vectors represented in ECEF coordinates [[meters]].
pub fn eci2ecef_j2000_batch(
    ecis: &[glam::DVec3],
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
) -> Vec<glam::DVec3> {
    return batch::map(ecis, eci2ecef_j2000_kernel(utc_time_str, utc_time_str_fmt));
}

/// Converts a batch of vectors from the Earth-rotation-only ECI frame to ECEF, sharing one UTC
/// time and writing into `out`. Matches [`eci2ecef_j2000`] applied to each point.
///
/// # Arguments
///
/// * `ecis` - Vectors represented in the rotated frame [[meters]].
/// * `utc_time_str` - UTC date and time, parsed once for the whole batch.
/// * `utc_time_str_fmt` - `chrono` format of `utc_time_str`, defaults to `%Y-%m-%dT%H:%M:%S`.
/// * `out` - Receives the vectors in ECEF coordinates [[meters]].
///
/// # Panics
///
/// If `ecis` and `out` differ in length.
pub fn eci2ecef_j2000_batch_into(
    ecis: &[glam::DVec3],
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
    out: &mut [glam::DVec3],
) {
    batch::map_into(
        ecis,
        out,
        eci2ecef_j2000_kernel(utc_time_str, utc_time_str_fmt),
    );
}

#[cfg(test)]
mod test_eci {
    use super::*;
    use rstest::*;

    /// Earth Rotation Angle at the J2000 epoch, IERS Conventions (2010) eq. 5.15 [[radians]].
    const ERA_J2000_RAD: f64 = std::f64::consts::TAU * 0.7790572732640;

    #[rstest]
    // J2000 epoch, by definition
    #[case((2000, 1, 1, 12, 0, 0), 2451545.0)]
    // Vallado, Fundamentals of Astrodynamics and Applications, Example 3-4
    #[case((1996, 10, 26, 14, 20, 0), 2450383.09722222)]
    // Meeus, Astronomical Algorithms, Example 7.a (Sputnik 1 launch, 1957 Oct 4.81)
    #[case((1957, 10, 4, 19, 26, 24), 2436116.31)]
    fn test_juliandate(#[case] utc: (i64, i64, i64, i64, i64, i64), #[case] expected: f64) {
        let (year, month, day, hour, min, sec) = utc;
        let actual = juliandate(year, month, day, hour, min, sec);
        assert!(
            f64::abs(actual - expected) < 1e-8,
            "juliandate{utc:?} = {actual}, expected {expected}"
        );
    }

    #[test]
    fn test_juliandate_from_utc_str() {
        let actual = juliandate_from_utc_str("2000-01-01T12:00:00".to_string(), None);
        assert_eq!(actual, 2451545.0);
        let actual = juliandate_from_utc_str(
            "1996/10/26 14:20".to_string(),
            Some("%Y/%m/%d %H:%M".to_string()),
        );
        assert!(f64::abs(actual - 2450383.09722222) < 1e-8, "{actual}");
    }

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

    #[rstest]
    #[case("2000-01-01T12:00:00", 0.0)]
    #[case("2000-01-02T12:00:00", 1.0)]
    #[case("2024-06-30T06:30:00", 8946.770833333333)]
    fn test_ecef2eci_j2000_earth_rotation_angle(#[case] utc: &str, #[case] days_since_j2000: f64) {
        let era =
            std::f64::consts::TAU * (0.7790572732640 + 1.00273781191135448 * days_since_j2000);
        let ecef = glam::DVec3::X * 6378137.0;
        let expected = glam::DVec3::new(f64::cos(era), f64::sin(era), 0.0) * 6378137.0;

        let eci = ecef2eci_j2000(&ecef, utc.to_string(), None);
        // A Julian date near 2.46e6 stored in an f64 resolves ~1e-10 days, which turns into ~1 cm
        // at the equator after the Earth rotation multiplies it up
        assert!(
            eci.abs_diff_eq(expected, 1e-2),
            "{utc}: eci {eci:?} expected {expected:?}"
        );
        let back = eci2ecef_j2000(&eci, utc.to_string(), None);
        assert!(back.abs_diff_eq(ecef, 1e-6), "{utc}: round trip {back:?}");
    }

    #[test]
    fn test_era_at_j2000_epoch() {
        let ecef = glam::DVec3::X;
        let eci = ecef2eci_j2000(&ecef, "2000-01-01T12:00:00".to_string(), None);
        let angle = f64::atan2(eci.y, eci.x).rem_euclid(std::f64::consts::TAU);
        assert!(
            f64::abs(angle - ERA_J2000_RAD) < 1e-12,
            "ERA {angle} rad, expected {ERA_J2000_RAD}"
        );
    }
}
