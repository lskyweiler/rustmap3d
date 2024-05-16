extern crate nalgebra_glm as glm;

static EARTH_SEMI_MAJOR_AXIS: f64 = 6378137.0;
static EARTH_SEMI_MAJOR_AXIS_2: f64 = EARTH_SEMI_MAJOR_AXIS * EARTH_SEMI_MAJOR_AXIS;
static EARTH_SEMI_MINOR_AXIS: f64 = 6356752.314245;
static EARTH_SEMI_MINOR_AXIS_2: f64 = EARTH_SEMI_MINOR_AXIS * EARTH_SEMI_MINOR_AXIS;
static EARTH_FLATTENING_FACTOR: f64 = 0.003352810664740;
static EARTH_ANGULAR_VEL_RADPS: f64 = 7.292115900000000e-05;
static EARTH_E: f64 = 521854.0084255785;
static EARTH_E_2: f64 = EARTH_E * EARTH_E;

static _ECEF2LLA_A: f64 = EARTH_SEMI_MAJOR_AXIS;
static _ECEF2LLA_B: f64 = _ECEF2LLA_A * (1.0 - EARTH_FLATTENING_FACTOR);
static _ECEF2LLA_A2: f64 = _ECEF2LLA_A * _ECEF2LLA_A;
static _ECEF2LLA_B2: f64 = _ECEF2LLA_B * _ECEF2LLA_B;
static _ECEF2LLA_E2: f64 = 1.0 - _ECEF2LLA_B2 / _ECEF2LLA_A2;
static _ECEF2LLA_EP2: f64 = _ECEF2LLA_A2 / _ECEF2LLA_B2 - 1.0;
static _ECEF2LLA_E2EP2: f64 = _ECEF2LLA_E2 * _ECEF2LLA_EP2;
static _ECEF2LLA_EP22: f64 = _ECEF2LLA_EP2 * _ECEF2LLA_EP2;
static _ECEF2LLA_EP24: f64 = _ECEF2LLA_EP22 * _ECEF2LLA_EP22;
static _ECEF2LLA_TWO_THRIDS: f64 = 2.0 / 3.0;

fn isclose(a: f64, b: f64, tol: f64) -> bool {
    return f64::abs(a - b) <= tol;
}

pub fn ecef2lla_ferarri(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let z_ecef_squared: f64 = f64::powf(z, 2.);
    let range_squared: f64 = f64::powf(x, 2.) + f64::powf(y, 2.);
    let range_: f64 = f64::sqrt(range_squared);
    let f_term: f64 = 54. * f64::powf(EARTH_SEMI_MINOR_AXIS, 2.) * z_ecef_squared;
    let g_term: f64 = range_squared + (1. - _ECEF2LLA_E2) * z_ecef_squared
        - _ECEF2LLA_E2
            * (f64::powf(EARTH_SEMI_MAJOR_AXIS, 2.) - f64::powf(EARTH_SEMI_MINOR_AXIS, 2.));

    let c_term: f64 = _ECEF2LLA_EP22 * f_term * range_squared / f64::powf(g_term, 3.);
    let c_term_sqrt_mod: f64 = f64::sqrt(f64::powf(c_term, 2.) + 2. * c_term);
    let s_term: f64 = f64::powf(1. + c_term + c_term_sqrt_mod, 1. / 3.);
    let p_term: f64 =
        f_term / (3. * f64::powf(s_term + 1. / s_term + 1., 2.) * f64::powf(g_term, 2.));
    let q_term: f64 = f64::sqrt(1. + 2. * _ECEF2LLA_EP22 * p_term);
    let sqrt_mod2: f64 = f64::sqrt(
        0.5 * f64::powf(EARTH_SEMI_MAJOR_AXIS, 2.) * (1. + 1. / q_term)
            - p_term * (1. - _ECEF2LLA_E2) * z_ecef_squared / (q_term * (1. + q_term))
            - 0.5 * p_term * range_squared,
    );
    let r0_term: f64 = -(p_term * _ECEF2LLA_E2 * range_) / (1. + q_term) + sqrt_mod2;
    let uv_subterm: f64 = f64::powf(range_ - _ECEF2LLA_E2 * r0_term, 2.);
    let u_term: f64 = f64::sqrt(uv_subterm + z_ecef_squared);
    let v_term: f64 = f64::sqrt(uv_subterm + (1. - _ECEF2LLA_E2) * z_ecef_squared);
    let z0_term: f64 = f64::powf(EARTH_SEMI_MINOR_AXIS, 2.) * z / (EARTH_SEMI_MAJOR_AXIS * v_term);
    let lat_rad: f64 = if range_ != 0. {
        f64::atan2(z + _ECEF2LLA_EP2 * z0_term, range_)
    } else {
        0.0
    };
    let lon_rad: f64 = f64::atan2(y, x);
    let alt: f64 =
        u_term * (1. - f64::powf(EARTH_SEMI_MINOR_AXIS, 2.) / (EARTH_SEMI_MAJOR_AXIS * v_term));

    let lat: f64 = f64::to_degrees(lat_rad);
    let lon: f64 = f64::to_degrees(lon_rad);
    return (lat, lon, alt);
}

pub fn ecef2lla_map3d(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let r = f64::sqrt(x * x + y * y + z * z);
    let r2 = r * r;
    let u = f64::sqrt(0.5 * (r2 - EARTH_E_2) + 0.5 * f64::hypot(r2 - EARTH_E_2, 2.0 * EARTH_E * z));
    let hxy = f64::hypot(x, y);
    let hue = f64::hypot(u, EARTH_E);

    let mut beta = std::f64::consts::FRAC_PI_2 * f64::signum(z);

    if !isclose(u, 0., 1e-6) && !isclose(hxy, 0., 1e-6) {
        beta = f64::atan(hue / u * z / hxy);
        beta += ((EARTH_SEMI_MINOR_AXIS * u - EARTH_SEMI_MAJOR_AXIS * hue + EARTH_E_2)
            * f64::sin(beta))
            / (EARTH_SEMI_MAJOR_AXIS * hue * 1. / f64::cos(beta) - EARTH_E_2 * f64::cos(beta))
    }

    let lat = f64::atan(EARTH_SEMI_MAJOR_AXIS / EARTH_SEMI_MINOR_AXIS * f64::tan(beta));
    let lon = f64::atan2(y, x);

    let mut alt = f64::hypot(
        z - EARTH_SEMI_MINOR_AXIS * f64::sin(beta),
        hxy - EARTH_SEMI_MAJOR_AXIS * f64::cos(beta),
    );
    let inside = x * x / EARTH_SEMI_MAJOR_AXIS_2
        + y * y / EARTH_SEMI_MAJOR_AXIS_2
        + z * z / EARTH_SEMI_MINOR_AXIS_2
        < 1.;
    alt *= if inside { -1. } else { 1.0 };
    return (f64::to_degrees(lat), f64::to_degrees(lon), alt);
}

pub fn ecef2lla(x: f64, y: f64, z: f64, use_ferarri: Option<bool>) -> (f64, f64, f64) {
    if use_ferarri.unwrap_or(false) {
        return ecef2lla_ferarri(x, y, z);
    } else {
        return ecef2lla_map3d(x, y, z);
    }
}

pub fn lla2ecef(lat: f64, lon: f64, alt: f64) -> (f64, f64, f64) {
    let lat = f64::to_radians(lat);
    let lon = f64::to_radians(lon);

    let alt_correction = EARTH_SEMI_MAJOR_AXIS_2
        / f64::hypot(
            EARTH_SEMI_MAJOR_AXIS * f64::cos(lat),
            EARTH_SEMI_MINOR_AXIS * f64::sin(lat),
        );
    let x = (alt_correction + alt) * f64::cos(lat) * f64::cos(lon);
    let y = (alt_correction + alt) * f64::cos(lat) * f64::sin(lon);
    let z = (alt_correction * f64::powf(EARTH_SEMI_MINOR_AXIS / EARTH_SEMI_MAJOR_AXIS, 2.0) + alt)
        * f64::sin(lat);
    return (x, y, z);
}

pub fn ecef2eci(x: f64, y: f64, z: f64, time_since_eci_lock: f64) -> (f64, f64, f64) {
    let earth_rotation_angle: f64 = time_since_eci_lock * EARTH_ANGULAR_VEL_RADPS;
    let mut rot: glm::DMat4 = glm::DMat4::identity();
    rot = glm::rotate_z(&rot, earth_rotation_angle);
    let eci: glm::DVec4 = rot * glm::DVec4::new(x, y, z, 1.);
    return (eci.x, eci.y, eci.z);
}
//
// fn enu2ecef_quat(lat: f64, lon: f64) -> glm::DQuat {
//     let lat_rad: f64 = f64::to_radians(lat);
//     let lon_rad: f64 = f64::to_radians(lon);
//
//     let mut dcm: glm::DQuat = glm::DQuat::identity();
//     dcm = glm::quat_rotate(&dcm, std::f64::consts::FRAC_PI_2 + lon_rad, &glm::DVec3::new(0., 0., 1.));
//     dcm = glm::quat_rotate(&dcm, std::f64::consts::FRAC_PI_2 - lat_rad, &glm::DVec3::new(1., 0., 0.));
//     return dcm;
// }
//
// pub fn enu2ecef_dcm(lat: f64, lon: f64) -> (f64, f64, f64, f64) {
//     let q = enu2ecef_quat(lat, lon);
//     return (q.i, q.j, q.k, q.w);
// }
//
// fn ecef2enu_quat(lat: f64, lon: f64) -> glm::DQuat {
//     let dcm: glm::DQuat = enu2ecef_quat(lat, lon);
//     let dcm: glm::DQuat = glm::quat_conjugate(&dcm);
//     return dcm;
// }
// pub fn ecef2enu_dcm(lat: f64, lon: f64) -> (f64, f64, f64, f64) {
//     let q = ecef2enu_quat(lat, lon);
//     return (q.i, q.j, q.k, q.w);
// }

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    fn test_ecef2lla_ferarri() {
        let actual = ecef2lla_ferarri(1., 2., 3.);
    }
}
