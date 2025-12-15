use map3d;
use pyo3::{exceptions::PyValueError, prelude::*};

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
    // pyglm serializes as w, x, y, z
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

#[pyfunction]
pub fn ecef2lla(x: f64, y: f64, z: f64) -> Vec3Tup {
    let lla = map3d::ecef2lla_map3d(&glam::DVec3::new(x, y, z));
    return vec3_to_tuple(&lla);
}
#[pyfunction]
pub fn lla2ecef(lat: f64, lon: f64, alt: f64) -> Vec3Tup {
    let ecef = map3d::lla2ecef(&glam::DVec3::new(lat, lon, alt));
    return vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn ecef2enu(x: f64, y: f64, z: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let enu = map3d::ecef2enu(
        &tuple_to_vec3(&(x, y, z)),
        &glam::DVec2::new(ref_lat, ref_lon),
    );
    return vec3_to_tuple(&enu);
}
#[pyfunction]
pub fn enu2ecef(e: f64, n: f64, u: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let ecef = map3d::enu2ecef(
        &tuple_to_vec3(&(e, n, u)),
        &glam::DVec2::new(ref_lat, ref_lon),
    );
    return vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn ecef2ned(x: f64, y: f64, z: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let ned = map3d::ecef2ned(
        &tuple_to_vec3(&(x, y, z)),
        &glam::DVec2::new(ref_lat, ref_lon),
    );
    return vec3_to_tuple(&ned);
}
#[pyfunction]
pub fn ned2ecef(n: f64, e: f64, d: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let ecef = map3d::ned2ecef(
        &tuple_to_vec3(&(n, e, d)),
        &glam::DVec2::new(ref_lat, ref_lon),
    );
    return vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn ecef2aer(x: f64, y: f64, z: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let aer = map3d::ecef2aer(
        &tuple_to_vec3(&(x, y, z)),
        &glam::DVec2::new(ref_lat, ref_lon),
    );
    return vec3_to_tuple(&aer);
}
#[pyfunction]
pub fn aer2ecef(a: f64, e: f64, r: f64, ref_lat: f64, ref_lon: f64, alt: f64) -> Vec3Tup {
    let ecef = map3d::aer2ecef(
        &tuple_to_vec3(&(a, e, r)),
        &glam::DVec3::new(ref_lat, ref_lon, alt),
    );
    return vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn enu2aer(e: f64, n: f64, u: f64) -> Vec3Tup {
    let aer = map3d::enu2aer(&tuple_to_vec3(&(e, n, u)));
    return vec3_to_tuple(&aer);
}
#[pyfunction]
pub fn aer2enu(a: f64, e: f64, r: f64) -> Vec3Tup {
    let enu = map3d::aer2enu(&tuple_to_vec3(&(a, e, r)));
    return vec3_to_tuple(&enu);
}
#[pyfunction]
pub fn ned2aer(n: f64, e: f64, d: f64) -> Vec3Tup {
    let aer = map3d::ned2aer(&tuple_to_vec3(&(n, e, d)));
    return vec3_to_tuple(&aer);
}
#[pyfunction]
pub fn aer2ned(a: f64, e: f64, r: f64) -> Vec3Tup {
    let ned = map3d::aer2enu(&tuple_to_vec3(&(a, e, r)));
    return vec3_to_tuple(&ned);
}

#[pyfunction]
pub fn rand_lla() -> Vec3Tup {
    let lla = map3d::rand_lla();
    return vec3_to_tuple(&lla);
}
#[pyfunction]
pub fn rand_ecef() -> Vec3Tup {
    let ecef = map3d::rand_ecef();
    return vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn rand_orientation() -> QuatTup {
    let quat = map3d::rand_orienation();
    return quat_to_tuple(&quat);
}

#[pyfunction]
pub fn orient_ecef_quat_towards_lla(
    obs_ecef: Vec3Tup,
    obs_ecef_quat: QuatTup,
    target_ecef: Vec3Tup,
) -> QuatTup {
    let ecef_quat = map3d::orient_ecef_quat_towards_lla(
        &tuple_to_vec3(&obs_ecef),
        &tuple_to_quat(&obs_ecef_quat),
        &tuple_to_vec3(&target_ecef),
    );
    return quat_to_tuple(&ecef_quat);
}

#[pyfunction]
pub fn ecef2enu_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::ecef2enu_quat(lat, lon);
    return quat_to_tuple(&quat);
}
#[pyfunction]
pub fn enu2ecef_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::enu2ecef_quat(lat, lon);
    return quat_to_tuple(&quat);
}
#[pyfunction]
pub fn ecef2ned_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::ecef2ned_quat(lat, lon);
    return quat_to_tuple(&quat);
}
#[pyfunction]
pub fn ned2ecef_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::ned2ecef_quat(lat, lon);
    return quat_to_tuple(&quat);
}
#[pyfunction]
pub fn ecef2enu_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::ecef2enu_dcm(lat, lon);
    return mat3_to_tuple(&dcm);
}
#[pyfunction]
pub fn enu2ecef_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::enu2ecef_dcm(lat, lon);
    return mat3_to_tuple(&dcm);
}
#[pyfunction]
pub fn ecef2ned_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::ecef2ned_dcm(lat, lon);
    return mat3_to_tuple(&dcm);
}
#[pyfunction]
pub fn ned2ecef_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::ned2ecef_dcm(lat, lon);
    return mat3_to_tuple(&dcm);
}

#[pyfunction]
pub fn enu2heading(east: f64, north: f64, up: f64) -> f64 {
    return map3d::enu2heading(&tuple_to_vec3(&(east, north, up)));
}
#[pyfunction]
pub fn ecef_quat2heading(ecef_quat: QuatTup, ref_lat: f64, ref_lon: f64) -> f64 {
    let heading = map3d::ecef_quat2heading(
        &tuple_to_quat(&ecef_quat),
        &glam::DVec2::new(ref_lat, ref_lon),
    );
    return heading;
}
#[pyfunction]
pub fn ecef2heading(ecef_rel: Vec3Tup, ref_lat: f64, ref_lon: f64) -> f64 {
    let heading = map3d::ecef2heading(
        &tuple_to_vec3(&ecef_rel),
        &glam::DVec2::new(ref_lat, ref_lon),
    );
    return heading;
}
#[pyfunction]
pub fn ecef2bearing(obs_ecef: Vec3Tup, targ_ecef: Vec3Tup) -> f64 {
    let bearing = map3d::ecef2bearing(
        &tuple_to_vec3(&obs_ecef),
        &tuple_to_vec3(&targ_ecef),
    );
    return bearing;
}

#[pyfunction]
pub fn angle_between(a: Vec3Tup, b: Vec3Tup) -> f64 {
    return map3d::angle_between_vec3(&tuple_to_vec3(&a), &tuple_to_vec3(&b));
}

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
