use map3d;
use map3d::{Vec3Tup, QuatTup, Mat3Tup};
use pyo3::prelude::*;
extern crate nalgebra_glm as glm;

#[pyfunction]
pub fn ecef2lla(x: f64, y: f64, z: f64) -> Vec3Tup {
    let lla = map3d::ecef2lla_map3d(&glm::DVec3::new(x, y, z));
    return map3d::vec3_to_tuple(&lla);
}
#[pyfunction]
pub fn lla2ecef(lat: f64, lon: f64, alt: f64) -> Vec3Tup {
    let ecef = map3d::lla2ecef(&glm::DVec3::new(lat, lon, alt));
    return map3d::vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn ecef2enu(x: f64, y: f64, z: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let enu = map3d::ecef2enu(
        &map3d::tuple_to_vec3(&(x, y, z)),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return map3d::vec3_to_tuple(&enu);
}
#[pyfunction]
pub fn enu2ecef(e: f64, n: f64, u: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let ecef = map3d::enu2ecef(
        &map3d::tuple_to_vec3(&(e, n, u)),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return map3d::vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn ecef2ned(x: f64, y: f64, z: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let ned = map3d::ecef2ned(
        &map3d::tuple_to_vec3(&(x, y, z)),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return map3d::vec3_to_tuple(&ned);
}
#[pyfunction]
pub fn ned2ecef(n: f64, e: f64, d: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let ecef = map3d::ned2ecef(
        &map3d::tuple_to_vec3(&(n, e, d)),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return map3d::vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn ecef2aer(x: f64, y: f64, z: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let aer = map3d::ecef2aer(
        &map3d::tuple_to_vec3(&(x, y, z)),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return map3d::vec3_to_tuple(&aer);
}
#[pyfunction]
pub fn aer2ecef(a: f64, e: f64, r: f64, ref_lat: f64, ref_lon: f64, alt: f64) -> Vec3Tup {
    let ecef = map3d::aer2ecef(
        &map3d::tuple_to_vec3(&(a, e, r)),
        &glm::DVec3::new(ref_lat, ref_lon, alt),
    );
    return map3d::vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn enu2aer(e: f64, n: f64, u: f64) -> Vec3Tup {
    let aer = map3d::enu2aer(&map3d::tuple_to_vec3(&(e, n, u)));
    return map3d::vec3_to_tuple(&aer);
}
#[pyfunction]
pub fn aer2enu(a: f64, e: f64, r: f64) -> Vec3Tup {
    let enu = map3d::aer2enu(&map3d::tuple_to_vec3(&(a, e, r)));
    return map3d::vec3_to_tuple(&enu);
}
#[pyfunction]
pub fn ned2aer(n: f64, e: f64, d: f64) -> Vec3Tup {
    let aer = map3d::ned2aer(&map3d::tuple_to_vec3(&(n, e, d)));
    return map3d::vec3_to_tuple(&aer);
}
#[pyfunction]
pub fn aer2ned(a: f64, e: f64, r: f64) -> Vec3Tup {
    let ned = map3d::aer2enu(&map3d::tuple_to_vec3(&(a, e, r)));
    return map3d::vec3_to_tuple(&ned);
}

#[pyfunction]
pub fn rand_lla() -> Vec3Tup {
    let lla = map3d::rand_lla();
    return map3d::vec3_to_tuple(&lla);
}
#[pyfunction]
pub fn rand_ecef() -> Vec3Tup {
    let ecef = map3d::rand_ecef();
    return map3d::vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn rand_orientation() -> QuatTup {
    let quat = map3d::rand_orienation();
    return map3d::quat_to_tuple(&quat);
}

#[pyfunction]
pub fn orient_ecef_quat_towards_lla(
    obs_ecef: Vec3Tup,
    obs_ecef_quat: QuatTup,
    target_ecef: Vec3Tup,
) -> QuatTup {
    let ecef_quat = map3d::orient_ecef_quat_towards_lla(
        &map3d::tuple_to_vec3(&obs_ecef),
        &map3d::tuple_to_quat(&obs_ecef_quat),
        &map3d::tuple_to_vec3(&target_ecef),
    );
    return map3d::quat_to_tuple(&ecef_quat);
}

#[pyfunction]
pub fn ecef2enu_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::ecef2enu_quat(lat, lon);
    return map3d::quat_to_tuple(&quat);
}
#[pyfunction]
pub fn enu2ecef_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::enu2ecef_quat(lat, lon);
    return map3d::quat_to_tuple(&quat);
}
#[pyfunction]
pub fn ecef2ned_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::ecef2ned_quat(lat, lon);
    return map3d::quat_to_tuple(&quat);
}
#[pyfunction]
pub fn ned2ecef_quat(lat: f64, lon: f64) -> QuatTup {
    let quat = map3d::ned2ecef_quat(lat, lon);
    return map3d::quat_to_tuple(&quat);
}
#[pyfunction]
pub fn ecef2enu_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::ecef2enu_dcm(lat, lon);
    return map3d::mat3_to_tuple(&dcm);
}
#[pyfunction]
pub fn enu2ecef_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::enu2ecef_dcm(lat, lon);
    return map3d::mat3_to_tuple(&dcm);
}
#[pyfunction]
pub fn ecef2ned_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::ecef2ned_dcm(lat, lon);
    return map3d::mat3_to_tuple(&dcm);
}
#[pyfunction]
pub fn ned2ecef_dcm(lat: f64, lon: f64) -> Mat3Tup {
    let dcm = map3d::ned2ecef_dcm(lat, lon);
    return map3d::mat3_to_tuple(&dcm);
}

#[pyfunction]
pub fn enu2heading(east: f64, north: f64, up: f64) -> f64 {
    return map3d::enu2heading(&map3d::tuple_to_vec3(&(east, north, up)));
}
#[pyfunction]
pub fn ecef_quat2heading(ecef_quat: QuatTup, ref_lat: f64, ref_lon: f64) -> f64 {
    let heading = map3d::ecef_quat2heading(
        &map3d::tuple_to_quat(&ecef_quat),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return heading;
}
#[pyfunction]
pub fn ecef2heading(ecef_rel: Vec3Tup, ref_lat: f64, ref_lon: f64) -> f64 {
    let heading = map3d::ecef2heading(
        &map3d::tuple_to_vec3(&ecef_rel),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return heading;
}
#[pyfunction]
pub fn ecef2bearing(obs_ecef: Vec3Tup, targ_ecef: Vec3Tup) -> f64 {
    let bearing = map3d::ecef2bearing(
        &map3d::tuple_to_vec3(&obs_ecef),
        &map3d::tuple_to_vec3(&targ_ecef),
    );
    return bearing;
}

#[pyfunction]
pub fn angle_between(a: Vec3Tup, b: Vec3Tup) -> f64 {
    return map3d::angle_between(&map3d::tuple_to_vec3(&a), &map3d::tuple_to_vec3(&b));
}

#[pymodule]
fn rustmap3d(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(ecef2lla, m)?)?;
    m.add_function(wrap_pyfunction!(lla2ecef, m)?)?;

    m.add_function(wrap_pyfunction!(ecef2enu, m)?)?;
    m.add_function(wrap_pyfunction!(enu2ecef, m)?)?;
    m.add_function(wrap_pyfunction!(ecef2ned, m)?)?;
    m.add_function(wrap_pyfunction!(ned2ecef, m)?)?;
    m.add_function(wrap_pyfunction!(ecef2aer, m)?)?;
    m.add_function(wrap_pyfunction!(aer2ecef, m)?)?;
    m.add_function(wrap_pyfunction!(enu2aer, m)?)?;
    m.add_function(wrap_pyfunction!(aer2enu, m)?)?;
    m.add_function(wrap_pyfunction!(ned2aer, m)?)?;
    m.add_function(wrap_pyfunction!(aer2ned, m)?)?;

    m.add_function(wrap_pyfunction!(rand_lla, m)?)?;
    m.add_function(wrap_pyfunction!(rand_ecef, m)?)?;
    m.add_function(wrap_pyfunction!(rand_orientation, m)?)?;

    m.add_function(wrap_pyfunction!(orient_ecef_quat_towards_lla, m)?)?;

    m.add_function(wrap_pyfunction!(ecef2enu_quat, m)?)?;
    m.add_function(wrap_pyfunction!(enu2ecef_quat, m)?)?;
    m.add_function(wrap_pyfunction!(ecef2ned_quat, m)?)?;
    m.add_function(wrap_pyfunction!(ned2ecef_quat, m)?)?;

    m.add_function(wrap_pyfunction!(ecef2enu_dcm, m)?)?;
    m.add_function(wrap_pyfunction!(enu2ecef_dcm, m)?)?;
    m.add_function(wrap_pyfunction!(ecef2ned_dcm, m)?)?;
    m.add_function(wrap_pyfunction!(ned2ecef_dcm, m)?)?;

    m.add_function(wrap_pyfunction!(ecef2bearing, m)?)?;
    m.add_function(wrap_pyfunction!(angle_between, m)?)?;
    Ok(())
}
