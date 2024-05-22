extern crate nalgebra_glm as glm;
use almost;
use chrono::{Datelike, NaiveDateTime, Timelike};
use rand;

use crate::util;

mod geo_const {
    pub static EARTH_SEMI_MAJOR_AXIS: f64 = 6378137.0;
    pub static EARTH_SEMI_MAJOR_AXIS_2: f64 = EARTH_SEMI_MAJOR_AXIS * EARTH_SEMI_MAJOR_AXIS;
    pub static EARTH_SEMI_MINOR_AXIS: f64 = 6356752.314245;
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

// todo: check performance with glam

pub fn ecef2lla_ferarri(ecef: &glm::DVec3) -> glm::DVec3 {
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
    return glm::DVec3::new(lat, lon, alt);
}
pub fn ecef2lla_map3d(ecef: &glm::DVec3) -> glm::DVec3 {
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
    return glm::DVec3::new(f64::to_degrees(lat), f64::to_degrees(lon), alt);
}
pub fn ecef2lla(ecef: &glm::DVec3) -> glm::DVec3 {
    return ecef2lla_map3d(ecef);
}

pub fn lla2ecef(lla: &glm::DVec3) -> glm::DVec3 {
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
    return glm::DVec3::new(x, y, z);
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
pub fn linear_velocity_from_earth_rotation(earth_pos: &glm::DVec3) -> glm::DVec3 {
    let rot_axis = glm::DVec3::new(0., 0., geo_const::EARTH_ANGULAR_VEL_RADPS);
    return glm::cross(&rot_axis, earth_pos);
}
pub fn ecef2eci(ecef: &glm::DVec3, time_since_eci_lock: f64) -> glm::DVec3 {
    let earth_rotation_angle: f64 = time_since_eci_lock * geo_const::EARTH_ANGULAR_VEL_RADPS;
    let eci = glm::rotate_z_vec3(ecef, earth_rotation_angle);
    return eci;
}
pub fn eci2ecef(eci: &glm::DVec3, time_since_eci_lock: f64) -> glm::DVec3 {
    let earth_rotation_angle: f64 = time_since_eci_lock * -geo_const::EARTH_ANGULAR_VEL_RADPS;
    let ecef = glm::rotate_z_vec3(eci, earth_rotation_angle);
    return ecef;
}
pub fn ecef2eci_j2000(
    ecef: &glm::DVec3,
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
) -> glm::DVec3 {
    let jd = juliandate_from_utc_str(utc_time_str, utc_time_str_fmt);
    let tu = jd - 2451545.0;
    let earth_rot_angle = std::f64::consts::TAU * (0.7790572732640 + 1.00273781191135448 * tu);
    return glm::rotate_z_vec3(ecef, earth_rot_angle);
}
pub fn eci2ecef_j2000(
    eci: &glm::DVec3,
    utc_time_str: String,
    utc_time_str_fmt: Option<String>,
) -> glm::DVec3 {
    let jd = juliandate_from_utc_str(utc_time_str, utc_time_str_fmt);
    let tu = jd - 2451545.0;
    let earth_rot_angle = std::f64::consts::TAU * (0.7790572732640 + 1.00273781191135448 * tu);
    return glm::rotate_z_vec3(eci, -earth_rot_angle);
}

pub fn enu2ecef_quat(lat: f64, lon: f64) -> glm::DQuat {
    let lat_rad: f64 = f64::to_radians(lat);
    let lon_rad: f64 = f64::to_radians(lon);

    let mut dcm: glm::DQuat = glm::DQuat::identity();
    dcm = glm::quat_rotate(
        &dcm,
        std::f64::consts::FRAC_PI_2 + lon_rad,
        &glm::DVec3::new(0., 0., 1.),
    );
    dcm = glm::quat_rotate(
        &dcm,
        std::f64::consts::FRAC_PI_2 - lat_rad,
        &glm::DVec3::new(1., 0., 0.),
    );
    return dcm;
}
pub fn ecef2enu_quat(lat: f64, lon: f64) -> glm::DQuat {
    let enu2ecef_rot = enu2ecef_quat(lat, lon);
    return enu2ecef_rot.conjugate();
}
pub fn ned2ecef_quat(lat: f64, lon: f64) -> glm::DQuat {
    let enu2ecef_q = enu2ecef_quat(lat, lon);
    let enu2ecef_mat = glm::quat_to_mat3(&enu2ecef_q);

    let east = glm::column(&enu2ecef_mat, 0);
    let north = glm::column(&enu2ecef_mat, 1);
    let up = glm::column(&enu2ecef_mat, 2);
    let down = -up;

    #[rustfmt::skip]
    let ned_mat = glm::DMat4::new(
        // new takes values in column major format for some reason
        north.x, east.x, down.x, 0.,  
        north.y, east.y, down.y, 0., 
        north.z, east.z, down.z, 0., 
        0., 0.,  0., 1.,
    );
    return glm::to_quat(&ned_mat);
}
pub fn ecef2ned_quat(lat: f64, lon: f64) -> glm::DQuat {
    let ned2ecef_rot = ned2ecef_quat(lat, lon);
    return ned2ecef_rot.conjugate();
}

pub fn enu2ecef_dcm(lat: f64, lon: f64) -> glm::DMat3 {
    let q = enu2ecef_quat(lat, lon);
    return glm::quat_to_mat3(&q);
}
pub fn ecef2enu_dcm(lat: f64, lon: f64) -> glm::DMat3 {
    let q = ecef2enu_quat(lat, lon);
    return glm::quat_to_mat3(&q);
}
pub fn ned2ecef_dcm(lat: f64, lon: f64) -> glm::DMat3 {
    let q = ned2ecef_quat(lat, lon);
    return glm::quat_to_mat3(&q);
}
pub fn ecef2ned_dcm(lat: f64, lon: f64) -> glm::DMat3 {
    let q = ecef2ned_quat(lat, lon);
    return glm::quat_to_mat3(&q);
}

/// Construct a quaternion that orients one ecef towards another ecef point that is tangent to the earth
/// Frame is initially aligned to the enu frame
/// Altitude difference is linearly inerpolated in pitch
///
/// Visualize sliding a playing card along a sphere where the playing card is the ecef coordiante frame
pub fn orient_ecef_quat_towards_lla(
    obs_ecef: &glm::DVec3,
    obs_ecef_quat: &glm::DQuat,
    target_ecef: &glm::DVec3,
) -> glm::DQuat {
    let obs2targ_ecef = target_ecef - obs_ecef;
    if glm::length(&obs2targ_ecef) == 0. {
        return obs_ecef_quat.clone();
    }
    let obs_lla = ecef2lla(&obs_ecef);

    let ecef2enu_dcm_at_obs = ecef2enu_dcm(obs_lla.x, obs_lla.y);
    let obs2targ_enu = ecef2enu_dcm_at_obs * obs2targ_ecef;
    let obs2targ_enu_dir = glm::normalize(&obs2targ_enu);

    let targ_lla = ecef2lla(&target_ecef);
    let alt_diff: f64 = targ_lla.z - obs_lla.z;

    let obs_enu_dcm = ecef2enu_dcm_at_obs * glm::quat_to_mat3(obs_ecef_quat);
    let obs_enu_forward = glm::column(&obs_enu_dcm, 0);

    // roll angle is based off east/north difference
    let en_diff = obs2targ_enu - obs_enu_forward;
    let mut roll_angle: f64 = util::angle_between(
        &glm::normalize(&obs2targ_enu.xy()),
        &glm::normalize(&obs_enu_forward.xy()),
    );
    roll_angle = f64::clamp(roll_angle, 0., std::f64::consts::FRAC_PI_2);
    roll_angle *= -f64::signum(en_diff.y);

    let yaw_angle: f64 = f64::atan2(obs2targ_enu_dir.y, obs2targ_enu_dir.x);
    let pitch_angle: f64 = f64::atan2(alt_diff, glm::length(&obs2targ_enu.xy()));

    // 321 Euler sequence
    let mut enu_quat = glm::DQuat::identity();
    enu_quat = glm::quat_rotate(&enu_quat, yaw_angle, &glm::DVec3::new(0., 0., 1.));
    enu_quat = glm::quat_rotate(&enu_quat, -pitch_angle, &glm::DVec3::new(0., 1., 0.));
    enu_quat = glm::quat_rotate(&enu_quat, roll_angle, &glm::DVec3::new(1., 0., 0.));

    // need to understand the math here better, quat * quat doens't equal mat3(quat) * mat3(quat) but it would be faster to directly multiply the quats
    // https://math.stackexchange.com/questions/331539/combining-rotation-quaternions#:~:text=To%20rotate%20a%20vector%20v,1%3Dvq%E2%80%B2q.
    let ecef_dcm = enu2ecef_dcm(obs_lla.x, obs_lla.y) * glm::quat_to_mat3(&enu_quat);
    let mut ecef_quat = glm::mat3_to_quat(&ecef_dcm);
    ecef_quat = glm::quat_normalize(&ecef_quat);
    return ecef_quat;
}

pub fn ecef2enu(ecef: &glm::DVec3, ref_lat_lon: &glm::DVec2) -> glm::DVec3 {
    let rot = ecef2enu_quat(ref_lat_lon.x, ref_lat_lon.y);
    return glm::quat_rotate_vec3(&rot, ecef);
}
pub fn enu2ecef(enu: &glm::DVec3, ref_lat_lon: &glm::DVec2) -> glm::DVec3 {
    let rot = enu2ecef_quat(ref_lat_lon.x, ref_lat_lon.y);
    return glm::quat_rotate_vec3(&rot, enu);
}
pub fn ecef2ned(ecef: &glm::DVec3, ref_lat_lon: &glm::DVec2) -> glm::DVec3 {
    let rot = ecef2ned_quat(ref_lat_lon.x, ref_lat_lon.y);
    return glm::quat_rotate_vec3(&rot, ecef);
}
pub fn ned2ecef(ned: &glm::DVec3, ref_lat_lon: &glm::DVec2) -> glm::DVec3 {
    let rot = ned2ecef_quat(ref_lat_lon.x, ref_lat_lon.y);
    return glm::quat_rotate_vec3(&rot, ned);
}
pub fn enu2aer(enu: &glm::DVec3) -> glm::DVec3 {
    let az = f64::atan2(enu.x, enu.y);
    let el = f64::atan2(enu.z, glm::length(&enu.xy()));
    let aer = glm::DVec3::new(f64::to_degrees(az), f64::to_degrees(el), glm::length(enu));
    return aer;
}
pub fn aer2enu(aer: &glm::DVec3) -> glm::DVec3 {
    let r = aer.z;
    let az = f64::to_radians(aer.x);
    let el = f64::to_radians(aer.y);
    let east_north = r * f64::cos(el);
    let east = east_north * f64::sin(az);
    let north = east_north * f64::cos(az);
    let up = r * f64::sin(el);
    return glm::DVec3::new(east, north, up);
}
pub fn ecef2aer(ecef: &glm::DVec3, ref_lat_lon: &glm::DVec2) -> glm::DVec3 {
    let enu = ecef2enu(ecef, ref_lat_lon);
    return enu2aer(&enu);
}
pub fn aer2ecef(aer: &glm::DVec3, ref_lla: &glm::DVec3) -> glm::DVec3 {
    let enu = aer2enu(aer);
    return enu2ecef(&enu, &ref_lla.xy()) + lla2ecef(ref_lla);
}
pub fn ned2aer(ned: &glm::DVec3) -> glm::DVec3 {
    let az = f64::atan2(ned.y, ned.x);
    let el = f64::atan2(-ned.z, glm::length(&ned.xy()));
    let aer = glm::DVec3::new(f64::to_degrees(az), f64::to_degrees(el), glm::length(ned));
    return aer;
}
pub fn aer2ned(aer: &glm::DVec3) -> glm::DVec3 {
    let enu = aer2enu(aer);
    return glm::DVec3::new(enu.y, enu.x, -enu.z);
}

pub fn enu2heading(enu: &glm::DVec3) -> f64 {
    let angle_off_east = f64::atan2(enu.y, enu.x);
    let mut heading = -angle_off_east + std::f64::consts::FRAC_PI_2;
    heading = f64::to_degrees(heading);
    return heading;
}
pub fn ecef_dcm2heading(ecef_dcm: &glm::DMat3, ref_lat_lon: &glm::DVec2) -> f64 {
    let ecef2enu = ecef2enu_dcm(ref_lat_lon.x, ref_lat_lon.y);
    let enu_dcm = ecef2enu * ecef_dcm;
    let forward = glm::column(&enu_dcm, 0);
    return enu2heading(&forward);
}
pub fn ecef_quat2heading(ecef_quat: &glm::DQuat, ref_lat_lon: &glm::DVec2) -> f64 {
    let ecef_dcm = glm::quat_to_mat3(ecef_quat);
    return ecef_dcm2heading(&ecef_dcm, ref_lat_lon);
}
pub fn ecef2heading(ecef_rel: &glm::DVec3, ref_lat_lon: &glm::DVec2) -> f64 {
    let enu = ecef2enu(ecef_rel, ref_lat_lon);
    return enu2heading(&enu);
}
pub fn ecef2bearing(obs_ecef: &glm::DVec3, targ_ecef: &glm::DVec3) -> f64 {
    let obs_lla = ecef2lla(obs_ecef);
    let diff = targ_ecef - obs_ecef;
    return ecef2heading(&diff, &obs_lla.xy());
}

pub fn rand_ecef() -> glm::DVec3 {
    let bounds: f64 = 1e7;
    return rand_ecef_in_range(-bounds, bounds, -bounds, bounds, -bounds, bounds);
}
pub fn rand_ecef_in_range(
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    z_min: f64,
    z_max: f64,
) -> glm::DVec3 {
    return glm::DVec3::new(
        glm::lerp_scalar(x_min, x_max, rand::random()),
        glm::lerp_scalar(y_min, y_max, rand::random()),
        glm::lerp_scalar(z_min, z_max, rand::random()),
    );
}
pub fn rand_lla() -> glm::DVec3 {
    return rand_lla_in_range(-90., 90., -180., 180., 0., 1e10);
}
pub fn rand_lla_in_range(
    lat_min: f64,
    lat_max: f64,
    lon_min: f64,
    lon_max: f64,
    alt_min: f64,
    alt_max: f64,
) -> glm::DVec3 {
    return glm::DVec3::new(
        glm::lerp_scalar(lat_min, lat_max, rand::random()),
        glm::lerp_scalar(lon_min, lon_max, rand::random()),
        glm::lerp_scalar(alt_min, alt_max, rand::random()),
    );
}

#[cfg(test)]
mod geotests {
    use super::*;
    use rstest::*;

    fn assert_vecs_close(a: &glm::DVec3, b: &glm::DVec3, tol: f64) {
        assert!(almost::equal_with(a.x, b.x, tol));
        assert!(almost::equal_with(a.y, b.y, tol));
        assert!(almost::equal_with(a.z, b.z, tol));
    }

    #[fixture]
    fn ecef_fixture() -> (Vec<glm::DVec3>, Vec<glm::DVec3>) {
        let ecefs: Vec<glm::DVec3> = vec![
            glm::DVec3::new(-4395937.137069274, -5292832.080918098, 269071.20488286763),
            glm::DVec3::new(-17425.064377540722, -2006865.517897409, 7930762.472554953),
            glm::DVec3::new(-3895837.31483683, 6644097.950743062, -1824545.3930313005),
            glm::DVec3::new(-6409133.120866153, 4558772.702384297, -1947061.2076449227),
            glm::DVec3::new(851485.3223891761, -2568595.6166770314, -8943557.577897426),
            glm::DVec3::new(5691809.8303289935, -7030732.321824082, -8330125.529865682),
            glm::DVec3::new(2516433.37963281, -1241849.192556627, -6760285.815142313),
            glm::DVec3::new(-3482763.8631693274, 6769587.309460899, -944640.6623275336),
            glm::DVec3::new(-370523.61786744185, 6024008.984975778, 8509531.670449838),
            glm::DVec3::new(5313476.383658681, -9825896.959403213, -1741391.5667025768),
            glm::DVec3::new(-8908541.706026632, -7037043.26505645, 6552883.707257189),
            glm::DVec3::new(-617141.4009794481, 2060754.5193012804, 8394970.341717768),
            glm::DVec3::new(8538245.25323937, 4747903.057412151, -9958019.246258922),
            glm::DVec3::new(-2108304.0336288405, -1654639.056214511, -9162254.95404531),
            glm::DVec3::new(-9223445.909886952, -400765.09915581346, -9900123.17700839),
            glm::DVec3::new(-7688902.383860592, -4471753.673588827, -5826850.3124411125),
            glm::DVec3::new(-3018693.051038252, -5388736.217988148, -5258460.323543303),
            glm::DVec3::new(-9288375.018009178, -2127487.6902046744, 6460020.61877924),
            glm::DVec3::new(-144971.8057209216, 114606.55915352888, 7971798.167930942),
            glm::DVec3::new(6828101.960543122, 693885.4050173741, -1196749.1985590179),
            glm::DVec3::new(-8578157.642460234, -1842531.7364011942, 6845094.434824787),
            glm::DVec3::new(5787354.780059811, 1210116.2836728748, -4726770.355619501),
            glm::DVec3::new(4193285.538259528, -1970166.3550874572, 7412301.116472796),
            glm::DVec3::new(3890132.340585826, 1161874.8197520673, -7909529.461251948),
            glm::DVec3::new(-7403258.820000752, -2903792.6257274374, -8826380.396521661),
            glm::DVec3::new(1857337.7916966896, 368102.4709657077, -6155560.052247517),
            glm::DVec3::new(-7588845.593718505, -8209442.364279032, 8265779.358689017),
            glm::DVec3::new(-5108159.12198603, -8476495.093040336, 6352145.996764505),
            glm::DVec3::new(-186001.42373996228, -7630898.02119923, -9721624.592627238),
            glm::DVec3::new(2369638.1324996054, 4073593.1322049536, 452913.52990143374),
            glm::DVec3::new(449824.9360387679, 4166398.4307631515, 2736272.9522982505),
            glm::DVec3::new(9965957.90734066, 2221724.6208532117, -6853904.619168903),
            glm::DVec3::new(-4478176.754602823, 1036869.5552951898, -9423720.838865533),
            glm::DVec3::new(2097024.4875351437, -9881162.142519716, 4949580.930534031),
            glm::DVec3::new(201204.94145997614, 9498512.914456967, 3515229.3174460693),
            glm::DVec3::new(-2060584.097762437, 3861553.560423618, -7614373.714063268),
            glm::DVec3::new(1757425.2753162049, 3398544.52313062, 8492215.282477114),
            glm::DVec3::new(-9047342.859610673, -8441792.192733321, -4469448.824678039),
            glm::DVec3::new(-487921.69639001414, -6030301.279856028, -9260258.22696507),
            glm::DVec3::new(-2744053.779757822, 2376587.0533640455, -3680945.978733376),
            glm::DVec3::new(-1815727.9597172216, 1953269.055585172, 6538261.825529313),
            glm::DVec3::new(8720682.736849941, -3489723.916849705, -5774441.696210405),
            glm::DVec3::new(-2933009.790774938, -5440069.166919639, -8623652.84907917),
            glm::DVec3::new(549449.791821098, -2920680.440199934, -3534971.5976333166),
            glm::DVec3::new(-5140412.492418254, -7429509.788924876, -1444380.0341157485),
            glm::DVec3::new(9626007.606757686, 8719399.041604526, 1697464.7512210696),
            glm::DVec3::new(-9127014.835284937, -6298505.842698975, -773826.4704206288),
            glm::DVec3::new(1842921.888856599, 8121796.140805811, 6227066.092612989),
            glm::DVec3::new(5138399.743888708, 9322686.007429656, -2555719.5912182704),
            glm::DVec3::new(9598896.75147951, 5044493.199291278, -6248509.982967714),
        ];
        let llas: Vec<glm::DVec3> = vec![
            glm::DVec3::new(2.253516806471046, -129.71116955184112, 507441.10318602197),
            glm::DVec3::new(75.87000157402547, -90.49747108091655, 1822723.22233205),
            glm::DVec3::new(-13.396850991057667, 120.38568018022004, 1538213.4642196773),
            glm::DVec3::new(-13.975248661411722, 144.5760168892664, 1725598.1936111697),
            glm::DVec3::new(-73.23828849316851, -71.65972404561161, 2985437.311158756),
            glm::DVec3::new(-42.74052199178427, -51.00770324601096, 5928794.414992734),
            glm::DVec3::new(-67.57498967210243, -26.26614362057283, 959684.6950130499),
            glm::DVec3::new(-7.112487646164081, 117.22455673918091, 1293518.4109493706),
            glm::DVec3::new(54.76454334396985, 93.51970396395149, 4068652.2450541486),
            glm::DVec3::new(-8.893647137325992, -61.59714074842926, 4927844.7237863885),
            glm::DVec3::new(30.07510912049895, -141.69405513524464, 6735317.147520137),
            glm::DVec3::new(75.69522425715692, 106.67156561450872, 2308138.501369687),
            glm::DVec3::new(-45.63516031581972, 29.077333155761565, 7582905.476120061),
            glm::DVec3::new(-73.76417124368244, -141.8744698912028, 3187753.7972916476),
            glm::DVec3::new(-47.08986703197784, -177.51202373059226, 7170110.832990964),
            glm::DVec3::new(-33.334219053050205, -149.81834070560083, 4261638.047698805),
            glm::DVec3::new(-40.55844825566602, -119.256926093939, 1742735.252764878),
            glm::DVec3::new(34.23382312320651, -167.09902807775964, 5140859.543690856),
            glm::DVec3::new(88.67910855596926, 141.67208580697599, 1617176.124676821),
            glm::DVec3::new(-9.951000598269978, 5.80259213124441, 589322.5756239324),
            glm::DVec3::new(38.067270677905, -167.87743403521839, 4758084.056677728),
            glm::DVec3::new(-38.79866773869741, 11.810196086116186, 1199912.009335559),
            glm::DVec3::new(58.11840077026344, -25.165937921189123, 2378386.964989038),
            glm::DVec3::new(-62.94048624737819, 16.62942891748623, 2529453.921751855),
            glm::DVec3::new(-48.084398711074584, -158.58331540347453, 5514132.784860136),
            glm::DVec3::new(-73.00848065406132, 11.210083031167493, 81603.365385446),
            glm::DVec3::new(36.56190250218874, -132.75043328084922, 7532975.424592685),
            glm::DVec3::new(32.78902393868271, -121.07427361795573, 5387958.701666567),
            glm::DVec3::new(-51.95818468723161, -91.39629526176816, 5995326.962900662),
            glm::DVec3::new(5.539216363414069, 59.81313129551833, -1643546.9126202406),
            glm::DVec3::new(33.367417431775394, 83.83793347365611, -1366875.6175801605),
            glm::DVec3::new(-33.963908882404155, 12.56752085132872, 5926171.518468513),
            glm::DVec3::new(-64.09003114845517, 166.96355233242235, 4124167.2847124636),
            glm::DVec3::new(26.190941042810294, -78.01820568227308, 4874709.392305468),
            glm::DVec3::new(20.38330525588493, 88.78649739621953, 3754550.8161831154),
            glm::DVec3::new(-60.228949370259244, 118.08506369211479, 2420676.065792122),
            glm::DVec3::new(65.84506256608404, 62.65601109433141, 2953957.9933096142),
            glm::DVec3::new(-19.918996193014078, -136.9830393254037, 6780862.795777614),
            glm::DVec3::new(-56.943542406173144, -94.62581967163953, 4698278.924516642),
            glm::DVec3::new(-45.63501439769118, 139.10461559970705, -1197409.76205113),
            glm::DVec3::new(67.93118253875696, 132.9100464016069, 701440.670716858),
            glm::DVec3::new(-31.68079101252089, -21.809626299383666, 4653736.780262814),
            glm::DVec3::new(-54.48102155235577, -118.3312980633975, 4245634.217446316),
            glm::DVec3::new(-50.206653953803524, -79.34581589819817, -1747312.1358781399),
            glm::DVec3::new(-9.125140350907303, -124.67901908506032, 2771588.2018704447),
            glm::DVec3::new(7.470140696990517, 42.17081118035693, 6720670.269056683),
            glm::DVec3::new(-4.007031763998265, -145.39064333427916, 4738278.426321056),
            glm::DVec3::new(36.898669089623674, 77.21546306700436, 4028403.093408107),
            glm::DVec3::new(-13.551349707628185, 61.137694236757405, 4570511.41330536),
            glm::DVec3::new(-30.036801286738232, 27.723202807325016, 6142375.721604907),
        ];
        return (ecefs, llas);
    }
    #[fixture]
    fn lla_fixture() -> (Vec<glm::DVec3>, Vec<glm::DVec3>) {
        let llas: Vec<glm::DVec3> = vec![
            glm::DVec3::new(-56.66172812377343, 155.68130312212668, 718875820.2943813),
            glm::DVec3::new(-33.26507252871781, -31.964572932142886, 7065343507.721147),
            glm::DVec3::new(6.371772880304988, -59.23405806377991, 8352922369.965223),
            glm::DVec3::new(-70.74335211522242, 120.07092031064208, 4526779918.14558),
            glm::DVec3::new(-24.461510525146508, -59.62407504106517, 2420545872.95108),
            glm::DVec3::new(-72.3152549895963, 101.64767964829839, 3795852142.154489),
            glm::DVec3::new(50.1705938605771, 105.40775429487171, 7390834021.2357435),
            glm::DVec3::new(-32.30257694254681, -135.9790191808388, 2106745558.3293736),
            glm::DVec3::new(-19.934988367833313, 65.33934397460283, 6927358798.01723),
            glm::DVec3::new(23.702048312802475, 150.7801192499238, 4151592272.73058),
            glm::DVec3::new(-44.139744893078216, -10.394871262469564, 1308050706.656194),
            glm::DVec3::new(80.33315640988386, -167.00766083959087, 4853517330.590896),
            glm::DVec3::new(89.5832033021365, -146.3731250375061, 5541861449.666565),
            glm::DVec3::new(6.002327772952086, 178.1723518113481, 8243112160.578481),
            glm::DVec3::new(-30.240836896769675, 5.48046771739655, 774183075.5164747),
            glm::DVec3::new(-66.50252282873791, -130.9736602675406, 6467951118.810617),
            glm::DVec3::new(32.244715738302816, 134.35132917429144, 8312562383.178297),
            glm::DVec3::new(-40.929722118941775, -166.0099481725839, 6216809556.871332),
            glm::DVec3::new(-3.161999721921518, -121.6362613721991, 1686482297.1376824),
            glm::DVec3::new(70.69015001986463, -18.77422907679727, 4347545178.616857),
            glm::DVec3::new(-77.89686967839751, 76.8590940393218, 275547474.5753206),
            glm::DVec3::new(56.70256197263234, 31.487362633295106, 3351651550.650465),
            glm::DVec3::new(-8.537166774081555, 124.9300397881874, 6964157346.925901),
            glm::DVec3::new(-3.3883037953841466, 159.9876304913834, 5824233554.097087),
            glm::DVec3::new(3.3173719339601604, 19.53837842565602, 7727952079.516271),
            glm::DVec3::new(-29.656043118692313, 50.423126817903324, 3109749957.585276),
            glm::DVec3::new(-3.317182434873658, 115.18183094259633, 6783718307.968782),
            glm::DVec3::new(87.99594613540816, 113.8812711654927, 6548397030.25793),
            glm::DVec3::new(42.155897640035505, -7.385067279532194, 4223681045.2455497),
            glm::DVec3::new(-45.66421911004391, 165.75779873561737, 9231088702.94254),
            glm::DVec3::new(74.98669309517001, -139.02652259218758, 7752650661.051985),
            glm::DVec3::new(0.04388642130096798, 139.12251298740722, 9856972131.89139),
            glm::DVec3::new(18.913840578810465, -152.22766889844678, 8674973608.740688),
            glm::DVec3::new(-70.71679219133352, 168.7938671723137, 2559962939.2985635),
            glm::DVec3::new(-17.840267641554703, 145.15346668909052, 9666183830.211079),
            glm::DVec3::new(7.81829152155791, 178.60319210639625, 2434544733.5319457),
            glm::DVec3::new(-86.60171868991569, -144.4301374069363, 8315174907.876453),
            glm::DVec3::new(-64.583522424254, 117.42569962019991, 2137778660.492624),
            glm::DVec3::new(-53.497097618994985, 66.73409062380492, 6968275498.757018),
            glm::DVec3::new(39.85170162247488, 82.86455367169816, 5083534728.543176),
            glm::DVec3::new(-89.14033256764529, 3.817737674123464, 8488834909.559485),
            glm::DVec3::new(-8.097151555351061, -60.04675219817315, 1794800252.0766892),
            glm::DVec3::new(60.35525557045381, 137.40735372191648, 2643745363.528964),
            glm::DVec3::new(-33.80752224309167, 151.33853754701425, 4511336080.320615),
            glm::DVec3::new(44.49769217327577, 35.65986371715448, 7747124179.421991),
            glm::DVec3::new(78.6881809933453, 153.6339574621485, 1262173554.861432),
            glm::DVec3::new(50.934753774135714, 48.68011259868561, 3940700238.395185),
            glm::DVec3::new(23.139059023558076, -30.9794825232527, 3732066418.3197985),
            glm::DVec3::new(-60.66725009582464, -27.905389704786387, 5654168566.610009),
            glm::DVec3::new(-89.52247206976813, 49.3889940119827, 2084170969.4723942),
        ];
        let ecefs: Vec<glm::DVec3> = vec![
            glm::DVec3::new(-363226337.8378485, 164145694.11941433, -605883225.5202017),
            glm::DVec3::new(5016417802.618961, -3130294546.8914347, -3878912812.734696),
            glm::DVec3::new(4249637013.1107903, -7138476691.558969, 927704883.3233159),
            glm::DVec3::new(-749123383.4568931, 1293818655.7673783, -4279508924.526832),
            glm::DVec3::new(1117071280.1290438, -1905836230.5036347, -1004929049.7808872),
            glm::DVec3::new(-233195557.85231522, 1131259987.718429, -3622523730.6242113),
            glm::DVec3::new(-1258810196.8083477, 4567666948.353717, 5680702139.465385),
            glm::DVec3::new(-1284356369.4257667, -1241197752.6029196, -1129213367.408031),
            glm::DVec3::new(2719703477.4990993, 5923770937.923891, -2364069442.8993587),
            glm::DVec3::new(-3322780364.3305306, 1858552546.759458, 1671407285.5992749),
            glm::DVec3::new(927816843.0914981, -170200408.00971267, -915359858.8829931),
            glm::DVec3::new(-795180643.7592813, -183469932.25196317, 4790867197.061249),
            glm::DVec3::new(-33606494.82719547, -22350830.551816523, 5548071401.239205),
            glm::DVec3::new(-8200090300.39463, 261659157.03025025, 862635420.7610741),
            glm::DVec3::new(671261500.4702597, 64404182.064525165, -393099795.1348588),
            glm::DVec3::new(-1692640892.2722268, -1948969436.132962, -5937439805.014645),
            glm::DVec3::new(-4918541001.068794, 5031191125.999227, 4438438970.340508),
            glm::DVec3::new(-4562247156.197555, -1136654630.154329, -4076992529.3376484),
            glm::DVec3::new(-886595545.8633595, -1439098934.8425846, -93374639.72836758),
            glm::DVec3::new(1363144037.4345145, -463368006.7734003, 4108967125.721511),
            glm::DVec3::new(13439868.383440318, 57568179.5971645, -275637077.97071016),
            glm::DVec3::new(1572069597.6960115, 962888601.5025085, 2806725105.8847656),
            glm::DVec3::new(-3946937207.6022162, 5651485039.359078, -1034776257.2819849),
            glm::DVec3::new(-5468975219.234577, 1991881397.6015773, -344601758.5886673),
            glm::DVec3::new(7276755652.2802105, 2582321037.9901214, 447557846.6049511),
            glm::DVec3::new(1725273328.2316494, 2087211973.2923207, -1541817056.8399067),
            glm::DVec3::new(-2884293340.9078116, 6134483975.098778, -392895716.5994358),
            glm::DVec3::new(-92799096.46533564, 209598007.20691597, 6550744575.743122),
            glm::DVec3::new(3109827878.675527, -403071979.06005263, 2838982707.422298),
            glm::DVec3::new(-6257302596.723574, 1588245806.4929829, -6607134865.92882),
            glm::DVec3::new(-1517523803.7477174, -1317930511.8018465, 7494157658.802485),
            glm::DVec3::new(-7457782245.210336, 6455006230.264203, 7554923.466331962),
            glm::DVec3::new(-7266578347.596589, -3826748677.7182827, 2814011879.0678635),
            glm::DVec3::new(-831350760.9757568, 164704360.3558542, -2422341250.456174),
            glm::DVec3::new(-7556421765.651863, 5260956590.476351, -2963316063.6143136),
            glm::DVec3::new(-2417514993.952572, 58948027.124627456, 332037521.0480978),
            glm::DVec3::new(-401231813.01364976, -286934665.0235125, -8306899057.677289),
            glm::DVec3::new(-423873876.7793667, 816838646.0451242, -1936605057.1374884),
            glm::DVec3::new(1638840880.6982732, 3811586986.433357, -5606389740.5359535),
            glm::DVec3::new(485379156.6587149, 3877298386.9417615, 3261608187.700436),
            glm::DVec3::new(127175105.33258279, 8486506.652578589, -8494235452.686916),
            glm::DVec3::new(890350698.4192543, -1545042792.5320225, -253693647.0788635),
            glm::DVec3::new(-965000269.7722342, 887134681.2380865, 2303222856.352874),
            glm::DVec3::new(-3293865707.0306754, 1800460317.3511906, -2513657365.399923),
            glm::DVec3::new(4493418846.659723, 3224078407.769552, 5434256241.694285),
            glm::DVec3::new(-222944108.97574744, 110505773.3995093, 1243887305.7373564),
            glm::DVec3::new(1642387205.8161027, 1868181939.908915, 3064602177.6773486),
            glm::DVec3::new(2947321316.0621667, -1769493145.9165955, 1469058917.9567044),
            glm::DVec3::new(2450561222.4285192, -1297800536.1928244, -4934781507.158352),
            glm::DVec3::new(11341319.896049185, 13227002.021572359, -2090455113.763534),
        ];
        return (llas, ecefs);
    }
    #[fixture]
    fn ecef2enu_fixture() -> (Vec<(glm::DVec3, glm::DVec2)>, Vec<glm::DVec3>) {
        let ecef_ref_tuple: Vec<(glm::DVec3, glm::DVec2)> = vec![
            (
                glm::DVec3::new(91619.6484735217, 6925836.249227896, 5957639.1833251435),
                glm::DVec2::new(-50.173150092148774, -112.00193826311323),
            ),
            (
                glm::DVec3::new(247424.65845609456, 6213457.926448222, -4561090.24503099),
                glm::DVec2::new(82.57782955532258, 179.8856930744933),
            ),
            (
                glm::DVec3::new(4192418.204552155, 7351021.414750714, 1979901.0055067968),
                glm::DVec2::new(62.457227071538824, 55.92173622713477),
            ),
            (
                glm::DVec3::new(-8588031.646098651, -7486734.193169072, -9230929.386199847),
                glm::DVec2::new(28.342454025165626, -92.0830991512501),
            ),
            (
                glm::DVec3::new(1427216.04466255, 9403604.322131746, -963231.1375281066),
                glm::DVec2::new(30.516610115227095, -89.90471953518953),
            ),
            (
                glm::DVec3::new(6642925.238247002, 5903029.863253074, -2232073.9836572986),
                glm::DVec2::new(45.400629146794955, -67.73807362936134),
            ),
            (
                glm::DVec3::new(2591477.518994638, -5409635.6294355355, -8285118.102197416),
                glm::DVec2::new(19.970663213164315, -93.2950875704390),
            ),
            (
                glm::DVec3::new(2528320.789589662, -3495684.1279134876, 8360309.7196244),
                glm::DVec2::new(66.11400734475339, -95.08388981259206),
            ),
            (
                glm::DVec3::new(-9735848.884452593, -8556065.641175192, 8333421.846976527),
                glm::DVec2::new(-14.691539720606187, -2.33006016905326),
            ),
            (
                glm::DVec3::new(7426961.41892605, 1655943.456426185, 6591322.351593178),
                glm::DVec2::new(-15.910515256034472, 165.4827913906837),
            ),
            (
                glm::DVec3::new(-3196405.2009189655, -9604287.523781441, -1060713.7649627458),
                glm::DVec2::new(-27.470280603087133, 102.2454141837),
            ),
            (
                glm::DVec3::new(7335138.4135223925, 2344465.657677945, 8588377.836181395),
                glm::DVec2::new(-10.5382655567836, -167.02758187958835),
            ),
            (
                glm::DVec3::new(7310959.32431601, 8350601.114886332, 6569499.540019801),
                glm::DVec2::new(63.317884611786326, 23.40930303068805),
            ),
            (
                glm::DVec3::new(-8640546.064591233, -2663666.0467856065, 1614399.1752727698),
                glm::DVec2::new(72.66364577543774, 163.0609809895735),
            ),
            (
                glm::DVec3::new(-8068883.573257133, -1893830.0686977436, 8266480.72264607),
                glm::DVec2::new(-25.23522156265527, 125.3010823588916),
            ),
            (
                glm::DVec3::new(-8234811.869950183, -3807444.077073443, 462885.92168885097),
                glm::DVec2::new(-64.46261342414945, 130.7529101785710),
            ),
            (
                glm::DVec3::new(5745089.68100643, 8186227.487702135, 1050884.6330531444),
                glm::DVec2::new(-64.03982768384915, 2.6480032202244956),
            ),
            (
                glm::DVec3::new(2393154.1929320674, 2067816.501552973, 3732474.8825010224),
                glm::DVec2::new(-60.74611032083097, 30.92056758445699),
            ),
            (
                glm::DVec3::new(1951197.9768817485, -7575128.269719007, 9939982.202217888),
                glm::DVec2::new(42.50213359486636, 113.81499658197713),
            ),
            (
                glm::DVec3::new(6216696.544619927, -2057542.5103991907, 1143021.5607241336),
                glm::DVec2::new(-30.03750128679215, -110.032118791896),
            ),
            (
                glm::DVec3::new(8262888.4250550615, 3106595.2889045794, 9812866.097362377),
                glm::DVec2::new(15.123933131017566, 164.52989386282826),
            ),
            (
                glm::DVec3::new(7218924.780765143, -447912.04118075967, 5171595.8773390055),
                glm::DVec2::new(81.52927937810878, -105.2611885755099),
            ),
            (
                glm::DVec3::new(-912999.7792285755, 2374762.7068727836, -4955536.282540757),
                glm::DVec2::new(10.831088217119586, 151.2254067023637),
            ),
            (
                glm::DVec3::new(3623182.097599946, -124808.5784964189, 6830241.830351334),
                glm::DVec2::new(71.11579335672388, -144.99222808538826),
            ),
            (
                glm::DVec3::new(2208932.904275298, -3444783.5638758554, -7103490.483277793),
                glm::DVec2::new(22.462213216531538, -26.6324481797208),
            ),
            (
                glm::DVec3::new(698713.3718823884, -8409581.988125127, -3903578.000227963),
                glm::DVec2::new(-87.47177257040715, -13.04761510545472),
            ),
            (
                glm::DVec3::new(-9556270.567275014, -7440669.805674921, 2815381.5153284855),
                glm::DVec2::new(7.966991562503196, 100.79184677036972),
            ),
            (
                glm::DVec3::new(499026.44389439, -1925421.3250966482, 7925563.273168109),
                glm::DVec2::new(-71.1002119000218, -138.06002686196018),
            ),
            (
                glm::DVec3::new(7238450.744717773, -4383306.046812624, -8262036.304761575),
                glm::DVec2::new(-13.118964393782036, 74.96561856267363),
            ),
            (
                glm::DVec3::new(-326376.61228840984, 7739298.466333255, -5871984.449330825),
                glm::DVec2::new(-72.49265123799492, 91.02269534514028),
            ),
            (
                glm::DVec3::new(-1226800.3167807497, 7604616.1196159385, 244802.38239366747),
                glm::DVec2::new(45.976409115548705, -86.172644141553),
            ),
            (
                glm::DVec3::new(-6183006.182703774, 1260934.9898499493, 5405250.482727632),
                glm::DVec2::new(-46.5078875763827, 96.5794699892873),
            ),
            (
                glm::DVec3::new(4892851.839336393, -4366090.555875951, -284396.9303730335),
                glm::DVec2::new(-81.70294712853254, -160.1769692625534),
            ),
            (
                glm::DVec3::new(9237967.029159822, 5403631.481081279, -3005393.284370361),
                glm::DVec2::new(-79.40102319570062, 38.14134828047784),
            ),
            (
                glm::DVec3::new(-6538766.213631488, -4940418.412632655, 2501053.290733097),
                glm::DVec2::new(13.527045111462556, 102.20616898781418),
            ),
            (
                glm::DVec3::new(-8784252.36675533, -8232153.597217414, 4884503.641264852),
                glm::DVec2::new(30.28635667737656, -169.57611510228907),
            ),
            (
                glm::DVec3::new(-4943652.169548029, -3855871.065791603, -4748617.896841602),
                glm::DVec2::new(74.73754290271745, 152.61923201200796),
            ),
            (
                glm::DVec3::new(2469337.744048135, -9190045.568734694, 4837736.320298016),
                glm::DVec2::new(-12.948807601309923, 46.97162855809245),
            ),
            (
                glm::DVec3::new(-1433624.4808392152, -182826.48301463015, -5022036.343881993),
                glm::DVec2::new(-45.74760712894212, 152.37356328703),
            ),
            (
                glm::DVec3::new(5344321.103997834, 2878251.6468978976, -2881088.089473922),
                glm::DVec2::new(30.33833424064295, -69.68258737927425),
            ),
            (
                glm::DVec3::new(6606169.345653623, 322403.8278391063, 1042680.5917547438),
                glm::DVec2::new(62.631186380765996, -139.94615572408208),
            ),
            (
                glm::DVec3::new(667958.4714587647, 8478848.064964425, 1694820.8740469385),
                glm::DVec2::new(-78.2754170386466, -114.12583810194599),
            ),
            (
                glm::DVec3::new(8174259.422747154, 2536147.3532497776, -9862214.372994114),
                glm::DVec2::new(71.51999174030132, -146.96589979434754),
            ),
            (
                glm::DVec3::new(-3472815.7693284587, 4576504.523164628, 5061670.1162975095),
                glm::DVec2::new(-2.043219531494401, 8.809342235384065),
            ),
            (
                glm::DVec3::new(4164265.524093671, -8876967.849787056, 5611143.587449348),
                glm::DVec2::new(-77.72886207380276, -58.502589304409916),
            ),
            (
                glm::DVec3::new(9676345.130538844, -6880885.856225148, 2825292.9134078994),
                glm::DVec2::new(18.694870013985096, 74.11457134859293),
            ),
            (
                glm::DVec3::new(8345975.870587062, 5602669.236328628, 6440366.360817695),
                glm::DVec2::new(-20.537448114795794, 80.45056404450543),
            ),
            (
                glm::DVec3::new(76565.8634340372, -5176255.943787364, 8534384.933999725),
                glm::DVec2::new(4.390777736040391, -163.5117863846071),
            ),
            (
                glm::DVec3::new(-8621776.079462763, 8582232.035568487, -7881480.052577838),
                glm::DVec2::new(70.39862590743721, -50.56334026059545),
            ),
            (
                glm::DVec3::new(-4399316.195656607, -6898512.909927797, 4952200.285973493),
                glm::DVec2::new(13.452784771177136, -80.37984986897065),
            ),
        ];
        let enu: Vec<glm::DVec3> = vec![
            glm::DVec3::new(-2509734.058866111, -1142229.8051993023, -8710086.829399489),
            glm::DVec3::new(-6213939.181029341, -356139.6129727494, -4553234.7863042625),
            glm::DVec3::new(646493.0548110288, -6565953.849297021, 5657224.674040573),
            glm::DVec3::new(-8310221.5498587, -11824484.096495166, 2477362.99531654),
            glm::DVec3::new(1442851.8614025046, 3944020.582333587, -8588097.243385006),
            glm::DVec3::new(8384083.9597701, 530705.873201302, -3658122.3308675354),
            glm::DVec3::new(2898130.522006808, -9580587.360908013, 2106248.00047252),
            glm::DVec3::new(2828141.7883684454, 406381.76914518373, 8963451.345762655),
            glm::DVec3::new(-8944812.308699876, 5682066.860649477, -11186750.260459974),
            glm::DVec3::new(-3464795.3083847975, 4481617.34349062, -8322123.193586318),
            glm::DVec3::new(5160745.024628614, -4957933.438608909, -7236716.489417127),
            glm::DVec3::new(-638024.4383018238, 7039962.2571193, -9115528.775503935),
            glm::DVec3::new(4758642.742106677, -6009149.6767303115, 10372387.808200765),
            glm::DVec3::new(5065559.227112598, -6668312.075854795, 3772818.7838625945),
            glm::DVec3::new(7679624.4540795535, 8806536.758325476, -704596.7317034043),
            glm::DVec3::new(8723624.845966201, 2447560.2355613476, 656383.0818598685),
            glm::DVec3::new(7912063.637985037, 5959948.937075286, 1732914.3346207177),
            glm::DVec3::new(544218.9974970771, 4542220.806800395, -1733919.4470265843),
            glm::DVec3::new(1273660.08066382, 12542690.992105417, 1025514.8480509501),
            glm::DVec3::new(6545395.75730307, 891176.5148664078, -742222.4719711064),
            glm::DVec3::new(-5198048.819521416, 11334531.338719131, -4327503.698459759),
            glm::DVec3::new(7082255.427083047, 2213827.8816959755, 4898931.340738286),
            glm::DVec3::new(-1642041.3457062577, -5232446.364934714, 977553.893898297),
            glm::DVec3::new(2180801.9347610455, 4950825.7295749895, 5525274.352594351),
            glm::DVec3::new(-2089105.6099524165, -7908981.413461881, 537723.1402286584),
            glm::DVec3::new(-8034727.96923753, 2404524.236063574, 4013552.0992562943),
            glm::DVec3::new(10780459.878230758, 3553258.2532735206, -5076247.431745518),
            glm::DVec3::new(1765741.0366824528, 3433493.892317093, -7201673.843672094),
            glm::DVec3::new(-8127704.602222981, -8581068.9141233, -418878.4728269605),
            glm::DVec3::new(188190.11501304168, 5618723.277314343, 7929564.744095197),
            glm::DVec3::new(-716453.8452096153, 5684941.331412287, -5153958.6238624565),
            glm::DVec3::new(5997804.751024207, 5142896.894642161, -2571615.2481815317),
            glm::DVec3::new(5766620.612909903, -3130671.465188509, -169145.9772780796),
            glm::DVec3::new(-1455502.2937485129, 9869173.857722817, 4904342.692741647),
            glm::DVec3::new(7435498.038448695, 3237763.4321128717, -2765632.3199936696),
            glm::DVec3::new(6506963.899765746, -890288.4638884775, 11209666.612828782),
            glm::DVec3::new(5697491.384008396, -3774239.402854111, -3892366.7913987376),
            glm::DVec3::new(-8076048.557717319, 3586898.887116836, -5989144.782130713),
            glm::DVec3::new(826761.0616518205, -2655410.2107246453, 4424343.6206899965),
            glm::DVec3::new(6011204.366063954, -2060480.221598107, -2183257.3166331653),
            glm::DVec3::new(4004336.8956559882, 5154200.514421189, -1494022.607362235),
            glm::DVec3::new(-2856049.652164667, -7499693.903157335, -3287403.620908837),
            glm::DVec3::new(2329930.4291646713, 4684682.772947516, -11964071.395589357),
            glm::DVec3::new(5054368.641846909, 4961083.740458888, -2909699.8362255576),
            glm::DVec3::new(-1087142.537981118, 10714673.301033907, -3411816.9755286165),
            glm::DVec3::new(-11190213.783338673, 3948581.5711958185, -2854530.496779622),
            glm::DVec3::new(-7300849.649118861, 8455065.135673758, 4211057.18533386),
            glm::DVec3::new(4985129.447896278, 8402484.675946508, 2044981.6104112952),
            glm::DVec3::new(-1207186.325554382, 8759529.854828134, -11485677.829091577),
            glm::DVec3::new(-5490298.462615342, 3405031.6482858593, 7051959.535886337),
        ];
        return (ecef_ref_tuple, enu);
    }
    #[fixture]
    fn ecef2ned_fixture() -> (Vec<(glm::DVec3, glm::DVec2)>, Vec<glm::DVec3>) {
        let ecef_ref_tuple: Vec<(glm::DVec3, glm::DVec2)> = vec![
            (
                glm::DVec3::new(6888437.030500963, 5159088.058806049, -1588568.383383099),
                glm::DVec2::new(-43.39498494726661, 4.058899692699072),
            ),
            (
                glm::DVec3::new(5675971.780695453, -3933745.478421451, -468060.9169528838),
                glm::DVec2::new(15.008767101905619, 146.92063867032067),
            ),
            (
                glm::DVec3::new(-4363243.112005923, 5116084.0831444785, 2367379.933506632),
                glm::DVec2::new(-44.90885855476071, 147.50865214856645),
            ),
            (
                glm::DVec3::new(6204344.719931791, 8043319.008791654, -3797048.613613347),
                glm::DVec2::new(41.36971468682316, 143.58178366847767),
            ),
            (
                glm::DVec3::new(-557145.6909457333, -7985975.838632684, -1316563.2909243256),
                glm::DVec2::new(19.959655219884283, 148.68397916564334),
            ),
            (
                glm::DVec3::new(-459804.4689456597, 7306198.5554328, -4790153.792160811),
                glm::DVec2::new(54.905008862344026, 17.53174938081216),
            ),
            (
                glm::DVec3::new(4394093.728079082, -2023529.1555146254, 6496899.54296466),
                glm::DVec2::new(30.26757622173315, -179.5885850468058),
            ),
            (
                glm::DVec3::new(7352055.509855617, -5121782.462257361, -3495912.7450521984),
                glm::DVec2::new(66.68482177955784, -111.21584705913939),
            ),
            (
                glm::DVec3::new(-5227681.427695597, 9350805.005802866, 6063589.3855974),
                glm::DVec2::new(-9.365477141597339, -151.03950532108723),
            ),
            (
                glm::DVec3::new(158812.85041147843, 8656676.484538134, -7818843.081377927),
                glm::DVec2::new(9.228104296299222, 74.36210755208026),
            ),
            (
                glm::DVec3::new(6289337.265826721, 805672.1394064799, 9276770.919476017),
                glm::DVec2::new(18.573413033048936, 31.542143103157088),
            ),
            (
                glm::DVec3::new(1925737.2316621258, -2301977.0805467907, 1513020.2832977697),
                glm::DVec2::new(-37.74068956750355, -111.81912172043178),
            ),
            (
                glm::DVec3::new(2255463.5973721338, 3133187.779792577, -469380.1598123852),
                glm::DVec2::new(-73.83161498479313, 92.73741190791725),
            ),
            (
                glm::DVec3::new(8467620.318925612, 6849204.462803648, 7963462.42715758),
                glm::DVec2::new(76.15483916763182, 14.615972981299592),
            ),
            (
                glm::DVec3::new(4105667.997088125, -4487317.573757457, 6232574.17015757),
                glm::DVec2::new(62.907473733546084, 142.21402827360305),
            ),
            (
                glm::DVec3::new(8995297.46464241, 1593900.2149121184, -988737.8673768956),
                glm::DVec2::new(28.84416815203001, 178.65282216728616),
            ),
            (
                glm::DVec3::new(5866501.682604484, -8352540.236067053, 2255662.100814244),
                glm::DVec2::new(-2.440043645549977, 46.85304254813022),
            ),
            (
                glm::DVec3::new(-5139287.558762874, 4629784.415816955, -7657314.13582964),
                glm::DVec2::new(-50.317103363790864, 106.04986981580731),
            ),
            (
                glm::DVec3::new(6318261.930673189, -7987849.595678076, -7072830.221753922),
                glm::DVec2::new(35.58071523442298, -163.71573556837956),
            ),
            (
                glm::DVec3::new(8200320.293980793, 683959.3652144801, 3611782.65124513),
                glm::DVec2::new(-85.19457696080306, 48.59996756812498),
            ),
            (
                glm::DVec3::new(1519058.9606308155, -2175811.8135434627, -2597201.19329625),
                glm::DVec2::new(86.49299711650835, -166.89886645986513),
            ),
            (
                glm::DVec3::new(9220625.604792222, -6300561.172051234, -7522096.711511366),
                glm::DVec2::new(-52.096228220403646, 108.26877252750512),
            ),
            (
                glm::DVec3::new(-9544348.486626834, -1487623.3606636561, -7969995.612516604),
                glm::DVec2::new(-43.21441983729025, -100.50146232612576),
            ),
            (
                glm::DVec3::new(-2994120.6520693544, -6393641.969406243, 72730.10419774428),
                glm::DVec2::new(-82.91183272475539, -143.668353171972),
            ),
            (
                glm::DVec3::new(-6012884.190658741, -2828893.973767963, 4631966.124507211),
                glm::DVec2::new(60.89878173481494, 150.6535423183193),
            ),
            (
                glm::DVec3::new(3452811.2714610514, 9330978.060863663, -8838981.123470027),
                glm::DVec2::new(31.71632117388809, 124.35285373258189),
            ),
            (
                glm::DVec3::new(-4986253.214297766, 1935827.8693882204, -1153719.326018421),
                glm::DVec2::new(-58.5324927987406, -10.21485056533632),
            ),
            (
                glm::DVec3::new(1382254.7904856037, 172002.60125266388, -3771079.9799958635),
                glm::DVec2::new(-25.712697133752684, 121.55802277283243),
            ),
            (
                glm::DVec3::new(1212004.377070479, -9751273.623413712, 4831487.548213271),
                glm::DVec2::new(-29.535020194777093, -163.54926231537002),
            ),
            (
                glm::DVec3::new(-5197391.84347292, 9062586.796555977, -2955488.769689851),
                glm::DVec2::new(-38.181975325847986, -50.68756898865132),
            ),
            (
                glm::DVec3::new(2674957.0449850517, 2421536.9123733453, 4312387.006029125),
                glm::DVec2::new(-20.156897643748977, -30.80952422019098),
            ),
            (
                glm::DVec3::new(-9969515.562865596, -6153809.175106484, -3311966.1867499687),
                glm::DVec2::new(-46.90512716652745, 49.463784406548086),
            ),
            (
                glm::DVec3::new(7508467.8342603445, 1363028.4182038382, -1711872.0663271137),
                glm::DVec2::new(-17.591926478565682, 72.65866461612316),
            ),
            (
                glm::DVec3::new(3243917.7794763483, -9064406.280864034, -1092956.2056234032),
                glm::DVec2::new(-43.339153779499895, -123.2328340359681),
            ),
            (
                glm::DVec3::new(-254687.97861935943, 1228098.5122885387, 5109695.345173651),
                glm::DVec2::new(69.09752776476617, -1.9502386648967445),
            ),
            (
                glm::DVec3::new(-662155.2929495294, 6180917.147207247, 7500326.629605424),
                glm::DVec2::new(56.234687825476634, -112.31953414170192),
            ),
            (
                glm::DVec3::new(2661775.1983660073, -8330658.996485413, 4511087.109226249),
                glm::DVec2::new(87.62786643692306, -35.345944003484306),
            ),
            (
                glm::DVec3::new(-3676457.2555731535, -5729506.758706078, 4346482.866220744),
                glm::DVec2::new(-89.5756383505163, 116.18330779130967),
            ),
            (
                glm::DVec3::new(-8044313.163986813, -7621922.104305084, 2985308.497923072),
                glm::DVec2::new(67.25768830206161, -79.20621240232589),
            ),
            (
                glm::DVec3::new(-7996386.218725819, 7078762.191946764, -2066076.453381911),
                glm::DVec2::new(-75.35782498171784, -81.10301636906564),
            ),
            (
                glm::DVec3::new(5846830.623713044, 7227198.072744723, -7331588.91594902),
                glm::DVec2::new(3.755795114555795, 54.281965733905416),
            ),
            (
                glm::DVec3::new(7437276.714211722, -4431803.69567279, -9628513.449088097),
                glm::DVec2::new(-82.68061073845303, 65.15883724004757),
            ),
            (
                glm::DVec3::new(8930051.083399918, 8768775.994698372, 8197023.54810205),
                glm::DVec2::new(-82.43918424587858, 89.6885364207107),
            ),
            (
                glm::DVec3::new(3107237.2934945915, 4247153.050324835, 8054203.0123866135),
                glm::DVec2::new(25.225415962780346, -45.91826532998783),
            ),
            (
                glm::DVec3::new(-5843117.926183505, 1742510.0939028692, -9822058.359018423),
                glm::DVec2::new(-62.81582870448219, -59.9729803092481),
            ),
            (
                glm::DVec3::new(4369988.455430791, -3234880.5994664277, 2410762.1663310328),
                glm::DVec2::new(-82.58346908888234, -121.01020355679265),
            ),
            (
                glm::DVec3::new(-4209382.927282661, -2104160.3402341865, 969685.9314502683),
                glm::DVec2::new(-37.18673973767942, -7.896719105632457),
            ),
            (
                glm::DVec3::new(-9034872.754234113, -6408263.019168887, 461004.63400196284),
                glm::DVec2::new(-77.24468086301746, -34.859107279766334),
            ),
            (
                glm::DVec3::new(-1705567.8205711525, -8011993.235225978, 8173151.087935612),
                glm::DVec2::new(-4.6791627952866435, 122.70539974596176),
            ),
            (
                glm::DVec3::new(-3126968.1268446445, -418269.6169602778, 3991905.823012369),
                glm::DVec2::new(-13.22364176207492, -71.31487816103186),
            ),
        ];
        let ned: Vec<glm::DVec3> = vec![
            glm::DVec3::new(3817222.48054082, 4658571.265151352, -6349553.538000638),
            glm::DVec3::new(1335560.479667212, 198199.46328847948, 6788746.964214704),
            glm::DVec3::new(6215046.4463947285, -1971463.5162006821, -2881531.5065581324),
            glm::DVec3::new(-2705751.452139706, -10155862.596903786, 2672791.8450811454),
            glm::DVec3::new(16941.438273651525, 7112108.151736968, 3903484.4614574388),
            glm::DVec3::new(-4196041.589283414, 7105335.751442247, 2906022.9995483346),
            glm::DVec3::new(7818661.838767107, 2055028.707839415, 507757.48807762563),
            glm::DVec3::new(-3325079.223590582, 8707243.769614134, 2373717.722279952),
            glm::DVec3::new(5990291.986467549, -10712799.23411021, 941094.0918214218),
            glm::DVec3::new(-9061359.615151051, 2180531.241850624, -7016738.689651038),
            glm::DVec3::new(6952053.224228817, -2603474.54049187, -8435296.427302884),
            glm::DVec3::new(2066450.2991287007, 2643374.4640652123, -197856.88363498566),
            glm::DVec3::new(2771666.85982101, -2402526.7977076243, -1292295.3383048982),
            glm::DVec3::new(-7728001.629512573, 4490844.479939028, -10106395.803587643),
            glm::DVec3::new(8174937.185333731, 1030751.23863034, -2818769.837445282),
            glm::DVec3::new(3454244.953396713, -1804943.7863353621, 8321289.964481875),
            glm::DVec3::new(2164974.005012828, -9992279.672711134, 2176239.0493952204),
            glm::DVec3::new(-371837.78150431626, 3658950.7480725506, -9641340.996500337),
            glm::DVec3::new(-3526752.99639549, 9439055.971421905, 7226156.516731704),
            glm::DVec3::new(6217719.1737010805, -5698837.223374292, 3101813.958870949),
            glm::DVec3::new(825608.3386346335, 2463504.6292084446, 2652671.676882206),
            glm::DVec3::new(-11622623.652663339, -6780805.413641624, -483987.5357060544),
            glm::DVec3::new(-3615814.874120512, -9113347.937575273, -7791098.351688512),
            glm::DVec3::new(6161623.468428223, 3376833.985513638, -692888.337145604),
            glm::DVec3::new(-1115448.4565121694, 5412718.962804998, -5922058.40516899),
            glm::DVec3::new(-10544501.6812377, -8115923.044843912, -248841.26316541713),
            glm::DVec3::new(-5080617.0732847275, 1020883.0769152861, 1756800.1702353738),
            glm::DVec3::new(-3647944.2084103096, -1267853.540811822, -1116381.595853447),
            glm::DVec3::new(4991934.067786062, 9695319.888885854, 990419.2709722328),
            glm::DVec3::new(-8693038.257175997, 1720354.0762511175, 6272937.350163244),
            glm::DVec3::new(4412558.882047785, 3449871.5464713047, 493587.2015961539),
            glm::DVec3::new(-10409383.191861024, 3577248.750890754, 5203629.587439036),
            glm::DVec3::new(-562179.2525078077, -6760908.88330678, -3890951.968104692),
            glm::DVec3::new(3188546.913593634, 7681057.538208658, -4971472.691921508),
            glm::DVec3::new(2099860.9212028678, 1218719.7314506723, -4667695.873833862),
            glm::DVec3::new(8712939.198567996, -2959883.4468468903, -3197008.9875583653),
            glm::DVec3::new(-6797822.957480535, -5255233.032677077, -4796557.332128665),
            glm::DVec3::new(-3487079.84736866, 5827317.746543763, 4372429.642127713),
            glm::DVec3::new(-4361519.839891313, -9329383.645262405, -5065222.912952845),
            glm::DVec3::new(-8485272.391840788, -6805383.920264025, 81490.94709643815),
            glm::DVec3::new(-7923794.00309282, -527825.6411444983, -8780939.872624164),
            glm::DVec3::new(-2116687.4840525235, -8610968.474886889, -9435735.063012568),
            glm::DVec3::new(9819083.582509633, -8882251.737492235, 6965604.174583564),
            glm::DVec3::new(7665132.586816506, 5186753.740633406, -2628069.0356712104),
            glm::DVec3::new(-8430208.39139448, -4186943.6469871406, -6712100.1414773455),
            glm::DVec3::new(827984.89422318, 5411990.781832159, 2323321.460391936),
            glm::DVec3::new(-1572839.8441506187, -2662525.2591259694, 3677474.7097341423),
            glm::DVec3::new(-3556595.327789179, -10422339.65689887, 1277791.7010225418),
            glm::DVec3::new(7671120.493029718, 5764204.045919783, 6467554.251532741),
            glm::DVec3::new(3747536.9393992727, -3096156.3708752114, 1502655.6004291987),
        ];
        return (ecef_ref_tuple, ned);
    }

    #[rstest]
    fn test_ecef2lla_ferarri(ecef_fixture: (Vec<glm::DVec3>, Vec<glm::DVec3>)) {
        let ecefs = ecef_fixture.0;
        let llas = ecef_fixture.1;
        for (i, ecef) in ecefs.iter().enumerate() {
            let actual_lla: glm::DVec3 = ecef2lla_ferarri(ecef);
            let expected_lla = llas.get(i).unwrap();
            assert_vecs_close(&actual_lla, expected_lla, 1e-6);
        }
    }
    #[rstest]
    fn test_ecef2lla_map3d(ecef_fixture: (Vec<glm::DVec3>, Vec<glm::DVec3>)) {
        let ecefs = ecef_fixture.0;
        let llas = ecef_fixture.1;
        for (i, ecef) in ecefs.iter().enumerate() {
            let actual_lla: glm::DVec3 = ecef2lla_map3d(ecef);
            let expected_lla = llas.get(i).unwrap();
            assert_vecs_close(&actual_lla, expected_lla, 1e-6);
        }
    }

    #[test]
    fn test_ecef2lla_zero_map3() {
        let zero = glm::DVec3::new(0., 0., 0.);
        let actual = ecef2lla_map3d(&zero);
        assert!(almost::zero(actual.x));
        assert!(almost::zero(actual.y));
        assert!(almost::equal_with(actual.z, -6378137.0, 1e-6));
    }
    #[test]
    fn test_ecef2lla_zero_ferrari() {
        let zero = glm::DVec3::new(0., 0., 0.);
        let actual = ecef2lla_ferarri(&zero);
        assert!(almost::zero(actual.x));
        assert!(almost::zero(actual.y));
        assert!(almost::equal_with(actual.z, -6292741.654585736, 1e-6));
    }

    #[rstest]
    fn test_lla2ecef(lla_fixture: (Vec<glm::DVec3>, Vec<glm::DVec3>)) {
        let llas = lla_fixture.0;
        let ecefs = lla_fixture.1;
        for (i, lla) in llas.iter().enumerate() {
            let actual_ecef = lla2ecef(lla);
            let expected_ecef = ecefs.get(i).unwrap();
            assert_vecs_close(&actual_ecef, expected_ecef, 1e-6);
        }
    }

    #[rstest]
    fn test_ecef2enu(ecef2enu_fixture: (Vec<(glm::DVec3, glm::DVec2)>, Vec<glm::DVec3>)) {
        let ecef_ref_tuples = ecef2enu_fixture.0;
        let enu = ecef2enu_fixture.1;
        for (i, ecef_ref_tuple) in ecef_ref_tuples.iter().enumerate() {
            let actual_enu = ecef2enu(&ecef_ref_tuple.0, &ecef_ref_tuple.1);
            let expected_enu = enu.get(i).unwrap();
            assert_vecs_close(&actual_enu, expected_enu, 1e-6);
        }
    }
    #[rstest]
    fn test_enu2ecef(ecef2enu_fixture: (Vec<(glm::DVec3, glm::DVec2)>, Vec<glm::DVec3>)) {
        let ecef_ref_tuples = ecef2enu_fixture.0;
        let enu = ecef2enu_fixture.1;
        for (i, ecef_ref_tuple) in ecef_ref_tuples.iter().enumerate() {
            let actual_ecef = enu2ecef(&enu.get(i).unwrap(), &ecef_ref_tuple.1);
            let expected_ecef = ecef_ref_tuple.0;
            assert_vecs_close(&actual_ecef, &expected_ecef, 1e-6);
        }
    }

    #[rstest]
    fn test_ecef2ned_dcm(){
        let actual = ecef2ned_dcm(0., 0.,);
        let ned = actual * glm::DVec3::new(0.5, -1., 1.);
        assert_vecs_close(&ned, &glm::DVec3::new(1., -1., -0.5), 1e-6);
    }
    #[rstest]
    fn test_ecef2ned(ecef2ned_fixture: (Vec<(glm::DVec3, glm::DVec2)>, Vec<glm::DVec3>)) {
        let ecef_ref_tuples = ecef2ned_fixture.0;
        let neds = ecef2ned_fixture.1;
        for (i, ecef_ref_tuple) in ecef_ref_tuples.iter().enumerate() {
            let actual_ned = ecef2ned(&ecef_ref_tuple.0, &ecef_ref_tuple.1);
            let expected_ned = neds.get(i).unwrap();
            assert_vecs_close(&actual_ned, expected_ned, 1e-6);
        }
    }
    #[rstest]
    fn test_ned2ecef(ecef2ned_fixture: (Vec<(glm::DVec3, glm::DVec2)>, Vec<glm::DVec3>)) {
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
        let actual_aer = enu2aer(&glm::DVec3::new(1000., 100., 10000.));
        let expected_aer =
            glm::DVec3::new(84.28940686250037, 84.26111457290625, 10050.373127401788);
        assert_vecs_close(&actual_aer, &expected_aer, 1e-6);
    }
    #[rstest]
    fn test_rae2enu() {
        let actual_enu = aer2enu(&glm::DVec3::new(15., 370., 123450.));
        let expected_enu =
            glm::DVec3::new(31465.800427043872, 117431.96589455023, 21436.8675329825);
        assert_vecs_close(&actual_enu, &expected_enu, 1e-6);
    }

    #[rstest]
    fn test_enu2heading() {
        {
            let actual_heading = enu2heading(&glm::DVec3::new(1000., 1000., 0.));
            let expected_heading = 45.;
            assert!(almost::equal_with(actual_heading, expected_heading, 1e-10));
        }
        {
            let actual_heading = enu2heading(&glm::DVec3::new(1000., -1000., 0.));
            let expected_heading = 135.;
            assert!(almost::equal_with(actual_heading, expected_heading, 1e-10));
        }
    }

    #[rstest]
    fn test_orient_ecef() {
        let obs_ecef = glm::DVec3::new(450230.78125, -5146161.5, 3728609.5);
        let obs_ecef_quat = glm::DQuat::new(
            0.8006407690254357,
            0.48038446141526137,
            0.32025630761017426,
            0.16012815380508713,
        );
        let target_ecef = glm::DVec3::new(356314.625, -5095536.5, 3823411.25);
        let oriented = orient_ecef_quat_towards_lla(&obs_ecef, &obs_ecef_quat, &target_ecef);
        let actual = glm::DQuat::new(
            -0.40894064679286724,
            0.06621115448490575,
            0.8652683273179942,
            -0.282301881259633,
        );
        assert!(almost::equal_with(actual.coords.x, oriented.coords.x, 1e-6));
        assert!(almost::equal_with(actual.coords.y, oriented.coords.y, 1e-6));
        assert!(almost::equal_with(actual.coords.z, oriented.coords.z, 1e-6));
        assert!(almost::equal_with(actual.w, oriented.w, 1e-6));
    }
    #[rstest]
    fn test_orient_ecef_no_nan() {
        let mut oriented = orient_ecef_quat_towards_lla(
            &glm::DVec3::new(6305378.849302336, 388308.7942886081, 875852.4072903388),
            &glm::DQuat::new(
                -0.5502415067503782,
                0.47887021402861657,
                -0.4489595996023603,
                0.5160938677122108,
            ),
            &glm::DVec3::new(6306774.13352542, 365852.079373571, 875850.3946216722),
        );
        assert!(!f64::is_nan(oriented.coords.x));
        assert!(!f64::is_nan(oriented.coords.y));
        assert!(!f64::is_nan(oriented.coords.z));
        assert!(!f64::is_nan(oriented.w));

        oriented = orient_ecef_quat_towards_lla(
            &glm::DVec3::new(6121830.402233266, 167012.75190830903, 1776355.8276747267),
            &glm::DQuat::new(
                0.5742464303807674,
                -0.430360480648345,
                0.4178937059909104,
                -0.5571317118817186,
            ),
            &glm::DVec3::new(6121975.8644942725, 162014.83423220483, 1776352.5269659779),
        );
        assert!(!f64::is_nan(oriented.coords.x));
        assert!(!f64::is_nan(oriented.coords.y));
        assert!(!f64::is_nan(oriented.coords.z));
        assert!(!f64::is_nan(oriented.w));
    }
}
