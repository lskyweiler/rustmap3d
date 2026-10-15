//! Test-only reference oracle.
//!
//! A line-for-line port of the C kinematics-library `src/transforms.c`, used as the ground truth for
//! validating the transforms in this crate. Like the C library, everything here works in
//! radians-radians-meters and AER azimuth is in [-pi, pi].
//!
//! The C library takes its constants from the `universal-constants` package, which is not
//! available here, so they are derived from the WGS84 defining parameters instead.

/// WGS84 constants derived from the defining parameters `a` and `1/f`.
pub mod wgs84 {
    pub const EARTH_R_EQ_M: f64 = 6378137.0;
    pub const EARTH_INV_FLATTENING: f64 = 298.257223563;
    pub const EARTH_FLATTENING: f64 = 1.0 / EARTH_INV_FLATTENING;
    pub const EARTH_R_POLAR_M: f64 = EARTH_R_EQ_M * (1.0 - EARTH_FLATTENING);
    pub const EARTH_R_EQ2_M2: f64 = EARTH_R_EQ_M * EARTH_R_EQ_M;
    pub const EARTH_R_POLAR2_M2: f64 = EARTH_R_POLAR_M * EARTH_R_POLAR_M;
    /// First eccentricity squared, e^2.
    pub const EARTH_ECC2: f64 = EARTH_FLATTENING * (2.0 - EARTH_FLATTENING);
    /// e^4, used by Ferrari's solution.
    pub const EARTH_ECC4: f64 = EARTH_ECC2 * EARTH_ECC2;
    /// Second eccentricity squared, e'^2.
    pub const EARTH_ECC2_P: f64 = EARTH_ECC2 / (1.0 - EARTH_ECC2);
}

use wgs84::*;

/// `transform_aer_to_ned`
pub fn aer_to_ned(aer: glam::DVec3) -> glam::DVec3 {
    let azimuth = aer.x;
    let elevation = aer.y;
    let range = aer.z;
    let north_east_projection = range * f64::cos(elevation);

    return glam::DVec3::new(
        north_east_projection * f64::cos(azimuth),
        north_east_projection * f64::sin(azimuth),
        -range * f64::sin(elevation),
    );
}

/// `transform_ecef_to_lla`
pub fn ecef_to_lla(ecef: glam::DVec3) -> glam::DVec3 {
    let r = f64::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
    let r2 = r * r;
    let z2 = ecef.z * ecef.z;
    let f = 54.0 * EARTH_R_POLAR2_M2 * z2;
    let g = r2 + (1.0 - EARTH_ECC2) * z2 - EARTH_ECC2 * (EARTH_R_EQ2_M2 - EARTH_R_POLAR2_M2);
    let c = EARTH_ECC4 * f * r2 / (g * g * g);
    let s = f64::powf(1.0 + c + f64::sqrt(c * c + 2.0 * c), 1.0 / 3.0);
    let p = f / (3.0 * f64::powf(s + 1.0 / s + 1.0, 2.0) * g * g);
    let q = f64::sqrt(1.0 + 2.0 * EARTH_ECC4 * p);

    let r0_term1 = -p * EARTH_ECC2 * r / (1.0 + q);
    let r0_term2 = 0.5 * EARTH_R_EQ2_M2 * (1.0 + 1.0 / q);
    let r0_term3 = p * (1.0 - EARTH_ECC2) * z2 / (q * (1.0 + q));
    let r0_term4 = 0.5 * p * r2;
    let r0 = r0_term1 + f64::sqrt(r0_term2 - r0_term3 - r0_term4);

    let u = f64::sqrt(f64::powf(r - EARTH_ECC2 * r0, 2.0) + z2);
    let v = f64::sqrt(f64::powf(r - EARTH_ECC2 * r0, 2.0) + (1.0 - EARTH_ECC2) * z2);
    let z0 = EARTH_R_POLAR2_M2 * ecef.z / (EARTH_R_EQ_M * v);

    let alt_m = u * (1.0 - EARTH_R_POLAR2_M2 / (EARTH_R_EQ_M * v));
    let lat_rad = f64::atan((ecef.z + EARTH_ECC2_P * z0) / r);
    let lon_rad = f64::atan2(ecef.y, ecef.x);

    return glam::DVec3::new(lat_rad, lon_rad, alt_m);
}

/// `transform_ecef_to_ned`
pub fn ecef_to_ned(ecef: glam::DVec3, lla_ref: glam::DVec3) -> glam::DVec3 {
    let ecef_ref = lla_to_ecef(lla_ref);
    return ecef_to_ned_local(ecef - ecef_ref, lla_ref);
}

/// `transform_ecef_to_ned_local`
pub fn ecef_to_ned_local(ecef: glam::DVec3, lla_ref: glam::DVec3) -> glam::DVec3 {
    let lat_rad = lla_ref.x;
    let lon_rad = lla_ref.y;
    let sin_lat = f64::sin(lat_rad);
    let cos_lat = f64::cos(lat_rad);
    let sin_lon = f64::sin(lon_rad);
    let cos_lon = f64::cos(lon_rad);

    return glam::DVec3::new(
        -sin_lat * cos_lon * ecef.x - sin_lat * sin_lon * ecef.y + cos_lat * ecef.z,
        -sin_lon * ecef.x + cos_lon * ecef.y,
        -cos_lat * cos_lon * ecef.x - cos_lat * sin_lon * ecef.y - sin_lat * ecef.z,
    );
}

/// `transform_lla_to_ecef`
pub fn lla_to_ecef(lla: glam::DVec3) -> glam::DVec3 {
    let lat_rad = lla.x;
    let lon_rad = lla.y;
    let alt_m = lla.z;
    let n = EARTH_R_EQ_M / f64::sqrt(1.0 - EARTH_ECC2 * f64::powf(f64::sin(lat_rad), 2.0));
    let xy_coeff = (n + alt_m) * f64::cos(lat_rad);

    return glam::DVec3::new(
        xy_coeff * f64::cos(lon_rad),
        xy_coeff * f64::sin(lon_rad),
        (n * (1.0 - EARTH_ECC2) + alt_m) * f64::sin(lat_rad),
    );
}

/// `transform_lla_to_ned`
pub fn lla_to_ned(lla: glam::DVec3, lla_ref: glam::DVec3) -> glam::DVec3 {
    return ecef_to_ned(lla_to_ecef(lla), lla_ref);
}

/// `transform_ned_to_aer`
pub fn ned_to_aer(ned: glam::DVec3) -> glam::DVec3 {
    let north_east_projection = f64::sqrt(f64::powf(ned.x, 2.0) + f64::powf(ned.y, 2.0));
    let range = ned.length();

    return glam::DVec3::new(
        f64::atan2(ned.y, ned.x),
        f64::atan2(-ned.z, north_east_projection),
        range,
    );
}

/// `transform_ned_to_ecef`
pub fn ned_to_ecef(ned: glam::DVec3, lla_ref: glam::DVec3) -> glam::DVec3 {
    return ned_to_ecef_local(ned, lla_ref) + lla_to_ecef(lla_ref);
}

/// `transform_ned_to_ecef_local`
pub fn ned_to_ecef_local(ned: glam::DVec3, lla_ref: glam::DVec3) -> glam::DVec3 {
    let sin_lat = f64::sin(lla_ref.x);
    let cos_lat = f64::cos(lla_ref.x);
    let sin_lon = f64::sin(lla_ref.y);
    let cos_lon = f64::cos(lla_ref.y);

    return glam::DVec3::new(
        -sin_lat * cos_lon * ned.x - sin_lon * ned.y - cos_lat * cos_lon * ned.z,
        -sin_lat * sin_lon * ned.x + cos_lon * ned.y - cos_lat * sin_lon * ned.z,
        cos_lat * ned.x - sin_lat * ned.z,
    );
}

/// `transform_ned_to_lla`
pub fn ned_to_lla(ned: glam::DVec3, lla_ref: glam::DVec3) -> glam::DVec3 {
    return ecef_to_lla(ned_to_ecef(ned, lla_ref));
}

/// Julian date per Vallado, *Fundamentals of Astrodynamics and Applications*, Algorithm 14.
/// Valid for years 1900 through 2100.
pub fn juliandate(year: i64, month: i64, day: i64, hour: i64, min: i64, sec: f64) -> f64 {
    // INT() in Vallado truncates toward zero; every quantity here is positive in the valid range
    let term_month = (7 * (year + (month + 9) / 12)) / 4;
    return (367 * year - term_month + (275 * month) / 9 + day) as f64
        + 1721013.5
        + ((sec / 60.0 + min as f64) / 60.0 + hour as f64) / 24.0;
}

/// Converts an LLA in [[degrees-degrees-meters]] to the oracle's [[radians-radians-meters]].
pub fn lla_deg2rad(lla_deg: glam::DVec3) -> glam::DVec3 {
    return glam::DVec3::new(lla_deg.x.to_radians(), lla_deg.y.to_radians(), lla_deg.z);
}

/// Converts an LLA in [[radians-radians-meters]] to [[degrees-degrees-meters]].
pub fn lla_rad2deg(lla_rad: glam::DVec3) -> glam::DVec3 {
    return glam::DVec3::new(lla_rad.x.to_degrees(), lla_rad.y.to_degrees(), lla_rad.z);
}
